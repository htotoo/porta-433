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
#include <cstddef>
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
    enum class SlicerKind : uint8_t {
        None = 0,
        PCM,
        PWM,
        Manchester,
        PPM,
        PIWM_RAW,
        PIWM_DC,
        DMC,
        OSV1,
        NRZS,
    };

    struct DecoderRef {
        const r_device* tmpl;
        uint16_t protocol_num;
        SlicerKind slicer;
    };
    static constexpr size_t kMaxDecoderRefs = 256;
    std::array<DecoderRef, kMaxDecoderRefs> devices_{};
    size_t ook_count_{0};
    size_t fm_count_{0};
    int16_t last_ook_hit_{-1};
    int16_t last_fm_hit_{-1};
    std::array<std::array<char, kMaxLineLen>, kMaxOutputLines> decoded_lines_{};
    size_t decoded_line_count_{0};
    pulse_data_t pulse_data_{};

    void append_line(const std::string& text) {
        if (decoded_line_count_ >= kMaxOutputLines) {
            return;
        }
        auto& dst = decoded_lines_[decoded_line_count_++];
        const size_t copy_len = std::min(dst.size() - 1, text.size());
        std::memcpy(dst.data(), text.data(), copy_len);
        dst[copy_len] = '\0';
    }

    static std::string value_to_string(const data_t* entry) {
        if (!entry) {
            return {};
        }

        switch (entry->type) {
            case DATA_INT:
                return to_string_dec_int(entry->value.v_int);
            case DATA_DOUBLE:
                if (entry->value.v_dbl != entry->value.v_dbl) {
                    return "nan";
                } else {
                    return to_string_decimal(static_cast<float>(entry->value.v_dbl), 1);
                }
            case DATA_STRING:
                return entry->value.v_ptr ? static_cast<const char*>(entry->value.v_ptr) : "";
            case DATA_DATA:
                return "{data}";
            case DATA_ARRAY:
                return "{array}";
            default:
                return {};
        }
    }

    static size_t append_data_entry_lines(ParserBridge* self, const data_t* entry, size_t& field_count, unsigned depth) {
        if (!self || !entry || !entry->key || field_count >= kMaxFieldsPerCallback) {
            return 0;
        }

        if (entry->type == DATA_DATA && entry->value.v_ptr && depth < 3) {
            size_t appended = 0;
            for (const data_t* child = static_cast<const data_t*>(entry->value.v_ptr);
                 child && field_count < kMaxFieldsPerCallback;
                 child = child->next) {
                appended += append_data_entry_lines(self, child, field_count, depth + 1);
            }
            return appended;
        }

        std::string line = entry->key;
        line += '=';
        line += value_to_string(entry);
        self->append_line(line);
        ++field_count;
        return 1;
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
        const size_t line_mark = self->decoded_line_count_;
        // self->append_line(decoder->name ? decoder->name : "?"); //no need for the long names
        size_t field_count = 0;
        size_t appended_fields = 0;
        for (const data_t* e = data; e && field_count < kMaxFieldsPerCallback; e = e->next) {
            appended_fields += append_data_entry_lines(self, e, field_count, 0);
        }
        if (appended_fields == 0) {
            self->decoded_line_count_ = line_mark;
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

        const size_t device_count = sizeof(available_devices) / sizeof(available_devices[0]);
        ook_count_ = 0;
        fm_count_ = 0;
        size_t total_count = 0;

        auto slicer_for_modulation = [](unsigned modulation) -> SlicerKind {
            switch (modulation) {
                case FSK_PULSE_PCM:
                case OOK_PULSE_PCM:
                    return SlicerKind::PCM;
                case FSK_PULSE_PWM:
                case OOK_PULSE_PWM:
                    return SlicerKind::PWM;
                case FSK_PULSE_MANCHESTER_ZEROBIT:
                case OOK_PULSE_MANCHESTER_ZEROBIT:
                    return SlicerKind::Manchester;
                case OOK_PULSE_PPM:
                    return SlicerKind::PPM;
                case OOK_PULSE_PIWM_RAW:
                    return SlicerKind::PIWM_RAW;
                case OOK_PULSE_PIWM_DC:
                    return SlicerKind::PIWM_DC;
                case OOK_PULSE_DMC:
                    return SlicerKind::DMC;
                case OOK_PULSE_PWM_OSV1:
                    return SlicerKind::OSV1;
                case OOK_PULSE_NRZS:
                    return SlicerKind::NRZS;
                default:
                    return SlicerKind::None;
            }
        };

        // First pass: OOK decoders.
        for (size_t i = 0; i < device_count && total_count < kMaxDecoderRefs; ++i) {
            const r_device* dev = available_devices[i];
            if (!dev || !dev->decode_fn || !dev->name || dev->modulation >= FSK_DEMOD_MIN_VAL) {
                continue;
            }
            const SlicerKind slicer = slicer_for_modulation(dev->modulation);
            if (slicer == SlicerKind::None) {
                continue;
            }
            devices_[total_count++] = DecoderRef{dev, static_cast<uint16_t>(i + 1), slicer};
            ++ook_count_;
        }

        // Second pass: FSK decoders (append after OOK).
        for (size_t i = 0; i < device_count && total_count < kMaxDecoderRefs; ++i) {
            const r_device* dev = available_devices[i];
            if (!dev || !dev->decode_fn || !dev->name || dev->modulation < FSK_DEMOD_MIN_VAL) {
                continue;
            }
            const SlicerKind slicer = slicer_for_modulation(dev->modulation);
            if (slicer == SlicerKind::None) {
                continue;
            }
            devices_[total_count++] = DecoderRef{dev, static_cast<uint16_t>(i + 1), slicer};
            ++fm_count_;
        }

        auto by_priority = [](const DecoderRef& a, const DecoderRef& b) {
            return a.tmpl->priority < b.tmpl->priority;
        };
        std::sort(devices_.begin(), devices_.begin() + static_cast<std::ptrdiff_t>(ook_count_), by_priority);
        std::sort(devices_.begin() + static_cast<std::ptrdiff_t>(ook_count_),
                  devices_.begin() + static_cast<std::ptrdiff_t>(ook_count_ + fm_count_),
                  by_priority);
    }

    int run_demods(pulse_data_t* pulse_data, bool fm_mode) {
        const size_t offset = fm_mode ? ook_count_ : 0;
        const size_t device_count = fm_mode ? fm_count_ : ook_count_;
        const DecoderRef* devices = devices_.data() + offset;
        if (device_count == 0) {
            return 0;
        }

        auto run_one = [this, pulse_data](const DecoderRef& ref) -> int {
            r_device dev = *ref.tmpl;
            dev.protocol_num = ref.protocol_num;
            dev.output_fn = &ParserBridge::output_handler;
            dev.output_ctx = this;
            dev.log_fn = &ParserBridge::log_handler;
            dev.decode_events = 0;
            dev.decode_ok = 0;
            dev.decode_messages = 0;
            std::memset(dev.decode_fails, 0, sizeof(dev.decode_fails));

            switch (ref.slicer) {
                case SlicerKind::PCM:
                    return pulse_slicer_pcm(pulse_data, &dev);
                case SlicerKind::PWM:
                    return pulse_slicer_pwm(pulse_data, &dev);
                case SlicerKind::Manchester:
                    return pulse_slicer_manchester_zerobit(pulse_data, &dev);
                case SlicerKind::PPM:
                    return pulse_slicer_ppm(pulse_data, &dev);
                case SlicerKind::PIWM_RAW:
                    return pulse_slicer_piwm_raw(pulse_data, &dev);
                case SlicerKind::PIWM_DC:
                    return pulse_slicer_piwm_dc(pulse_data, &dev);
                case SlicerKind::DMC:
                    return pulse_slicer_dmc(pulse_data, &dev);
                case SlicerKind::OSV1:
                    return pulse_slicer_osv1(pulse_data, &dev);
                case SlicerKind::NRZS:
                    return pulse_slicer_nrzs(pulse_data, &dev);
                case SlicerKind::None:
                default:
                    return 0;
            }
        };

        int16_t& last_hit = fm_mode ? last_fm_hit_ : last_ook_hit_;
        if (last_hit >= 0 && static_cast<size_t>(last_hit) < device_count) {
            int events = run_one(devices[static_cast<size_t>(last_hit)]);
            if (events > 0) {
                return events;
            }
        }

        int events = 0;
        unsigned current_priority = devices[0].tmpl->priority;
        for (size_t idx = 0; idx < device_count; ++idx) {
            if (last_hit >= 0 && idx == static_cast<size_t>(last_hit)) {
                continue;
            }

            const DecoderRef& ref = devices[idx];
            const unsigned dev_priority = ref.tmpl->priority;
            if (dev_priority != current_priority) {
                if (events > 0) {
                    break;
                }
                current_priority = dev_priority;
            }

            events = run_one(ref);
            if (events > 0) {
                last_hit = static_cast<int16_t>(idx);
                return events;
            }
        }

        last_hit = -1;
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
    };

    options_modulation.on_change = [this](size_t, OptionsField::value_t v) {
        modulation_ = static_cast<uint8_t>(v);
        baseband::set_subghzd_config(modulation_, receiver_model.sampling_rate());
        console.writeln(modulation_ ? "mode: FM/FSK" : "mode: AM/OOK");
    };

    field_frequency.set_step(10000);

    baseband::run_image(portapack::spi_flash::image_tag_rtl433);
    options_modulation.set_by_value(modulation_ ? 1 : 0);
    baseband::set_subghzd_config(modulation_, receiver_model.sampling_rate());
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

    if (result.events <= 0 || result.line_count == 0) {
        return;
    }

    console.writeln("------------------------------");
    for (size_t i = 0; i < result.line_count; ++i) {
        console.writeln(parser_bridge_->line_at(i));
    }
}

}  // namespace ui
