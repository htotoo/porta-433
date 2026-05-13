/*
 * Copyright (C) 2014 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2017 Furrtek
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui_rtl433.hpp"

#include "baseband_api.hpp"
#include "string_format.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <climits>
#include <cstring>
#include <string>

extern "C" {
#include "data.h"
#include "pulse_data.h"
#include "pulse_slicer.h"
#include "r_device.h"
#include "rtl_433_devices.h"

#define DECL(name) extern r_device name;
DEVICES
#undef DECL
}

using namespace portapack;
using namespace ui;

namespace ui {

class RTL433View::ParserBridge {
   public:
    ParserBridge() {
        init_devices();
    }

    struct ParseResult {
        int events{0};
        size_t line_count{0};
    };

    ParseResult parse(const RtlPulsePacketData* packet, bool fm_mode) {
        ParseResult result{};
        decoded_line_count_ = 0;

        if (!packet || packet->num_pulses == 0 || packet->sample_rate == 0) {
            return result;
        }

        pulse_data_t& pulse_data = pulse_data_;
        std::memset(&pulse_data, 0, sizeof(pulse_data));
        pulse_data.sample_rate = packet->sample_rate;
        pulse_data.num_pulses = std::min<unsigned>(packet->num_pulses, PD_MAX_PULSES);
        pulse_data.ook_low_estimate = static_cast<int>(packet->ook_low_estimate);
        pulse_data.ook_high_estimate = static_cast<int>(packet->ook_high_estimate);

        for (unsigned i = 0; i < pulse_data.num_pulses; ++i) {
            pulse_data.pulse[i] = static_cast<int>(packet->pulse[i]);
            pulse_data.gap[i] = static_cast<int>(packet->gap[i]);
        }

        result.events = run_demods(&pulse_data, fm_mode);
        result.line_count = decoded_line_count_;
        return result;
    }

    const char* line_at(size_t index) const {
        if (index >= decoded_line_count_) {
            return "";
        }
        return decoded_lines_[index].data();
    }

   private:
    static constexpr size_t kMaxFieldsPerCallback = 48;
    static constexpr size_t kMaxOutputLines = 24;
    static constexpr size_t kMaxLineLen = 96;
    struct DecoderRef {
        const r_device* tmpl;
        unsigned protocol_num;
    };
    std::vector<DecoderRef> devices_{};
    std::array<std::array<char, kMaxLineLen>, kMaxOutputLines> decoded_lines_{};
    size_t decoded_line_count_{0};
    pulse_data_t pulse_data_{};

    void append_line(const char* text) {
        if (!text || decoded_line_count_ >= kMaxOutputLines) {
            return;
        }
        auto& dst = decoded_lines_[decoded_line_count_++];
        std::snprintf(dst.data(), dst.size(), "%s", text);
    }

    static void value_to_string(data_t* entry, char* out, size_t out_len) {
        if (!out || out_len == 0) {
            return;
        }
        out[0] = '\0';

        if (!entry) {
            return;
        }

        switch (entry->type) {
            case DATA_INT:
                std::snprintf(out, out_len, "%d", entry->value.v_int);
                return;
            case DATA_DOUBLE:
                std::snprintf(out, out_len, "%.3f", entry->value.v_dbl);
                return;
            case DATA_STRING:
                std::snprintf(out, out_len, "%s", entry->value.v_ptr ? static_cast<const char*>(entry->value.v_ptr) : "");
                return;
            case DATA_DATA:
                std::snprintf(out, out_len, "{data}");
                return;
            case DATA_ARRAY:
                std::snprintf(out, out_len, "{array}");
                return;
            default:
                return;
        }
    }

    /* Arena-allocated data is always valid – no pointer-sanity checks needed. */
    static void output_handler(r_device* decoder, data_t* data) {
        if (!decoder || !data) {
            if (data) data_free(data);
            return;
        }
        auto* self = static_cast<ParserBridge*>(decoder->output_ctx);
        if (!self) {
            data_free(data);
            return;
        }
        self->append_line(decoder->name ? decoder->name : "?");
        size_t field_count = 0;
        for (data_t* e = data; e && field_count < kMaxFieldsPerCallback; e = e->next, ++field_count) {
            if (!e->key) continue;
            char value_buf[48]{};
            char line_buf[kMaxLineLen]{};
            value_to_string(e, value_buf, sizeof(value_buf));
            std::snprintf(line_buf, sizeof(line_buf), "%s=%s", e->key, value_buf);
            self->append_line(line_buf);
        }
        data_free(data);
    }

    static void log_handler(r_device*, int, data_t* data) {
        if (data) {
            data_free(data);
        }
    }

    void init_devices() {
#define DECL(name) &name,
        static const r_device* const available_devices[] = {DEVICES};
#undef DECL

        devices_.reserve(sizeof(available_devices) / sizeof(available_devices[0]));

        for (size_t i = 0; i < (sizeof(available_devices) / sizeof(available_devices[0])); ++i) {
            const r_device* dev = available_devices[i];
            if (!dev || !dev->decode_fn || !dev->name) {
                continue;
            }

            devices_.push_back(DecoderRef{dev, static_cast<unsigned>(i + 1)});
        }
    }

    int run_demods(pulse_data_t* pulse_data, bool fm_mode) {
        int events = 0;
        unsigned next_priority = 0;

        /* Keep rtl_433's original behavior: stop scanning priorities after first hit.
         * This prevents long UI stalls when many decoders would otherwise continue to run. */
        for (unsigned priority = 0; priority < UINT_MAX && events == 0; priority = next_priority) {
            next_priority = UINT_MAX;

            for (const auto& ref : devices_) {
                const r_device* dev_tmpl = ref.tmpl;
                if (dev_tmpl->priority > priority && dev_tmpl->priority < next_priority) {
                    next_priority = dev_tmpl->priority;
                }
                if (dev_tmpl->priority != priority) {
                    continue;
                }

                /* Early exit if modulation doesn't match (cheap check on const template). */
                bool mod_match = false;
                if (fm_mode) {
                    mod_match = (dev_tmpl->modulation >= FSK_DEMOD_MIN_VAL);
                } else {
                    mod_match = (dev_tmpl->modulation < FSK_DEMOD_MIN_VAL);
                }
                if (!mod_match) {
                    continue;
                }

                /* Only copy and decode if modulation already matched. */
                r_device dev = *dev_tmpl;
                dev.protocol_num = ref.protocol_num;
                dev.output_fn = &ParserBridge::output_handler;
                dev.output_ctx = this;
                dev.log_fn = &ParserBridge::log_handler;
                dev.decode_events = 0;
                dev.decode_ok = 0;
                dev.decode_messages = 0;
                std::memset(dev.decode_fails, 0, sizeof(dev.decode_fails));

                switch (dev.modulation) {
                    case FSK_PULSE_PCM:
                    case OOK_PULSE_PCM:
                        events += pulse_slicer_pcm(pulse_data, &dev);
                        break;
                    case FSK_PULSE_PWM:
                    case OOK_PULSE_PWM:
                        events += pulse_slicer_pwm(pulse_data, &dev);
                        break;
                    case FSK_PULSE_MANCHESTER_ZEROBIT:
                    case OOK_PULSE_MANCHESTER_ZEROBIT:
                        events += pulse_slicer_manchester_zerobit(pulse_data, &dev);
                        break;
                    case OOK_PULSE_PPM:
                        events += pulse_slicer_ppm(pulse_data, &dev);
                        break;
                    case OOK_PULSE_PIWM_RAW:
                        events += pulse_slicer_piwm_raw(pulse_data, &dev);
                        break;
                    case OOK_PULSE_PIWM_DC:
                        events += pulse_slicer_piwm_dc(pulse_data, &dev);
                        break;
                    case OOK_PULSE_DMC:
                        events += pulse_slicer_dmc(pulse_data, &dev);
                        break;
                    case OOK_PULSE_PWM_OSV1:
                        events += pulse_slicer_osv1(pulse_data, &dev);
                        break;
                    case OOK_PULSE_NRZS:
                        events += pulse_slicer_nrzs(pulse_data, &dev);
                        break;
                    default:
                        break;
                }
            }
        }

        return events;
    }
};

RTL433View::RTL433View(NavigationView& nav)
    : nav_{nav},
      parser_bridge_{std::make_unique<ParserBridge>()} {
    add_children({&rssi,
                  &channel,
                  &field_rf_amp,
                  &field_lna,
                  &field_vga,
                  &field_frequency,
                  &button_clear,
                  &options_modulation,
                  &console});

    button_clear.on_select = [this](Button&) {
        console.clear(true);
        console.writeln("rtl_433 pulse console cleared");
    };

    options_modulation.on_change = [this](size_t, OptionsField::value_t v) {
        modulation_ = static_cast<uint8_t>(v);
        baseband::set_subghzd_config(modulation_, receiver_model.sampling_rate());
        console.writeln(modulation_ ? "mode: FM/FSK" : "mode: AM/OOK");
    };

    field_frequency.set_step(10000);

    console.writeln("rtl_433 pulse receiver ready");
    console.writeln("waiting for RtlPulsePacketData from M4...");

    baseband::run_image(portapack::spi_flash::image_tag_rtl433);
    options_modulation.set_by_value(modulation_ ? 1 : 0);
    baseband::set_subghzd_config(modulation_, receiver_model.sampling_rate());
    console.writeln(modulation_ ? "mode: FM/FSK" : "mode: AM/OOK");
    receiver_model.enable();
}

RTL433View::~RTL433View() {
    receiver_model.disable();
    baseband::shutdown();
}

void RTL433View::focus() {
    field_frequency.focus();
}

void RTL433View::on_freqchg(int64_t freq) {
    field_frequency.set_value(freq);
}

void RTL433View::on_packet(const RtlPulsePacketData* packet) {
    if (packet == nullptr || packet->num_pulses == 0) {
        return;
    }

    append_decoded_results(packet);
}

void RTL433View::append_decoded_results(const RtlPulsePacketData* packet) {
    if (!parser_bridge_) {
        return;
    }

    const auto result = parser_bridge_->parse(packet, modulation_ != 0);

    if (result.events <= 0) {
        return;
    }

    console.writeln("------------------------------");
    console.writeln(
        "F=" + to_string_freq(receiver_model.target_frequency()) +
        " parsed=" + to_string_dec_uint(static_cast<uint32_t>(result.events)));

    for (size_t i = 0; i < result.line_count; ++i) {
        console.writeln(parser_bridge_->line_at(i));
    }
}

}  // namespace ui
