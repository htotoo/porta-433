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
#include <climits>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "data.h"
#include "pulse_data.h"
#include "pulse_slicer.h"
#include "r_device.h"
#include "rtl_433_devices.h"

#define DEVICESMINIMAL \
    DECL(lacrossetx)   \
    DECL(acurite_th)   \
    DECL(nexus)        \
    DECL(acurite_txr)  \
    DECL(acurite_986)

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
        std::vector<std::string> lines{};
    };

    ParseResult parse(const RtlPulsePacketData* packet, bool fm_mode) {
        ParseResult result{};
        decoded_lines_.clear();

        if (!packet || packet->num_pulses == 0 || packet->sample_rate == 0) {
            return result;
        }

        pulse_data_t pulse_data{};
        pulse_data.sample_rate = packet->sample_rate;
        pulse_data.num_pulses = std::min<unsigned>(packet->num_pulses, PD_MAX_PULSES);
        pulse_data.ook_low_estimate = static_cast<int>(packet->ook_low_estimate);
        pulse_data.ook_high_estimate = static_cast<int>(packet->ook_high_estimate);

        for (unsigned i = 0; i < pulse_data.num_pulses; ++i) {
            pulse_data.pulse[i] = static_cast<int>(packet->pulse[i]);
            pulse_data.gap[i] = static_cast<int>(packet->gap[i]);
        }

        result.events = run_demods(&pulse_data, fm_mode);
        result.lines = decoded_lines_;
        return result;
    }

   private:
    std::vector<r_device> devices_{};
    std::vector<std::string> decoded_lines_{};

    static void output_handler(r_device* decoder, data_t* data) {
        if (!decoder || !data) {
            return;
        }

        auto* self = static_cast<ParserBridge*>(decoder->output_ctx);
        if (!self) {
            data_free(data);
            return;
        }

        std::array<char, 384> json{};
        data_print_jsons(data, json.data(), json.size());
        self->decoded_lines_.push_back(std::string(decoder->name) + " " + json.data());
        data_free(data);
    }

    static void log_handler(r_device*, int, data_t* data) {
        if (data) {
            data_free(data);
        }
    }

    void init_devices() {
#define DECL(name) &name,
        static r_device const* const available_devices[] = {DEVICESMINIMAL};
#undef DECL

        devices_.reserve(sizeof(available_devices) / sizeof(available_devices[0]));

        for (size_t i = 0; i < (sizeof(available_devices) / sizeof(available_devices[0])); ++i) {
            const r_device* src = available_devices[i];
            if (!src || !src->decode_fn || !src->name) {
                continue;
            }
            if (std::strcmp(src->name, "Template") == 0 || std::strcmp(src->name, "New Template") == 0 || std::strcmp(src->name, "Template decoder") == 0) {
                continue;
            }

            r_device device = *src;
            device.protocol_num = static_cast<unsigned>(i + 1);
            device.output_fn = &ParserBridge::output_handler;
            device.output_ctx = this;
            device.log_fn = &ParserBridge::log_handler;
            device.decode_events = 0;
            device.decode_ok = 0;
            device.decode_messages = 0;
            std::memset(device.decode_fails, 0, sizeof(device.decode_fails));
            devices_.push_back(device);
        }
    }

    int run_demods(pulse_data_t* pulse_data, bool fm_mode) {
        int events = 0;
        unsigned next_priority = 0;

        for (unsigned priority = 0; !events && priority < UINT_MAX; priority = next_priority) {
            next_priority = UINT_MAX;

            for (auto& dev : devices_) {
                if (dev.priority > priority && dev.priority < next_priority) {
                    next_priority = dev.priority;
                }
                if (dev.priority != priority) {
                    continue;
                }

                if (fm_mode) {
                    switch (dev.modulation) {
                        case FSK_PULSE_PCM:
                            events += pulse_slicer_pcm(pulse_data, &dev);
                            break;
                        case FSK_PULSE_PWM:
                            events += pulse_slicer_pwm(pulse_data, &dev);
                            break;
                        case FSK_PULSE_MANCHESTER_ZEROBIT:
                            events += pulse_slicer_manchester_zerobit(pulse_data, &dev);
                            break;
                        default:
                            break;
                    }
                } else {
                    switch (dev.modulation) {
                        case OOK_PULSE_PCM:
                            events += pulse_slicer_pcm(pulse_data, &dev);
                            break;
                        case OOK_PULSE_PPM:
                            events += pulse_slicer_ppm(pulse_data, &dev);
                            break;
                        case OOK_PULSE_PWM:
                            events += pulse_slicer_pwm(pulse_data, &dev);
                            break;
                        case OOK_PULSE_MANCHESTER_ZEROBIT:
                            events += pulse_slicer_manchester_zerobit(pulse_data, &dev);
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

    for (const auto& line : result.lines) {
        console.writeln(line);
    }
}

}  // namespace ui
