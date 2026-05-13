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

using namespace portapack;
using namespace ui;

namespace ui {

RTL433View::RTL433View(NavigationView& nav)
    : nav_{nav} {
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

    append_packet_summary(packet);
}

void RTL433View::append_packet_summary(const RtlPulsePacketData* packet) {
    console.writeln("------------------------------");
    console.writeln(
        "F=" + to_string_freq(receiver_model.target_frequency()) +
        " SR=" + to_string_dec_uint(packet->sample_rate) +
        " N=" + to_string_dec_uint(packet->num_pulses));

    console.writeln(
        "OOK lo=" + to_string_dec_uint(packet->ook_low_estimate) +
        " hi=" + to_string_dec_uint(packet->ook_high_estimate));

    std::string line{};
    line.reserve(80);

    for (uint16_t i = 0; i < packet->num_pulses && i < RtlPulsePacketData::max_pulses; ++i) {
        line += "P" + to_string_dec_uint(i) + ":" + to_string_dec_uint(packet->pulse[i]);
        line += " G:" + to_string_dec_uint(packet->gap[i]) + "  ";

        if ((i % 3) == 2) {
            console.writeln(line);
            line.clear();
        }
    }

    if (!line.empty()) {
        console.writeln(line);
    }
}

}  // namespace ui
