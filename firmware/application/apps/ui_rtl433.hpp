/*
 * Copyright (C) 2026 HTotoo
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

#ifndef __UI_RTL433_H__
#define __UI_RTL433_H__

#include "app_settings.hpp"
#include "radio_state.hpp"
#include "ui.hpp"
#include "ui_freq_field.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"

#include <memory>

namespace ui {

class RTL433View : public View {
   public:
    RTL433View(NavigationView& nav);
    ~RTL433View() override;

    void focus() override;
    std::string title() const override { return "RTL433"; }

   private:
    class ParserBridge;
    void on_tick_second();
    void on_freqchg(int64_t freq);
    void on_packet(const RtlPulsePacketData* packet);
    void append_decoded_results(const RtlPulsePacketData* packet);
    SignalToken signal_token_tick_second{};
    NavigationView& nav_;
    RxRadioState radio_state_{
        433'920'000 /* frequency */,
        1'750'000 /* bandwidth */,
        1'750'000 /* sampling rate */,
        ReceiverModel::Mode::AMAudio};

    uint8_t modulation_{0};  // 0 = AM/OOK, 1 = FM/FSK

    app_settings::SettingsManager settings_{
        "rx_rtl433",
        app_settings::Mode::RX,
        {
            {"modulation"sv, &modulation_},
        }};

    RFAmpField field_rf_amp{{13 * 8, UI_POS_Y(0)}};
    LNAGainField field_lna{{15 * 8, UI_POS_Y(0)}};
    VGAGainField field_vga{{18 * 8, UI_POS_Y(0)}};
    RSSI rssi{{21 * 8, 0, UI_POS_WIDTH_REMAINING(24), 4}};
    // Channel channel{{21 * 8, 5, UI_POS_WIDTH_REMAINING(24), 4}};

    RxFrequencyField field_frequency{
        {UI_POS_X(0), UI_POS_Y(0)},
        nav_};

    Button button_clear{
        {0, 16, 7 * 8, 16},
        "Clear"};

    OptionsField options_modulation{
        {10 * 8, 18},
        3,
        {{"AM", 0}, {"FM", 1}}};

    ActivityDot status_frame{
        {UI_POS_X_RIGHT(2) + 2, 10, 4, 4},
        Theme::getInstance()->bg_darkest->foreground,
    };

    Console console{{0, 32, screen_width, screen_height - 32 - 16}};

    std::unique_ptr<ParserBridge> parser_bridge_{};

    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};

    MessageHandlerRegistration message_handler_packet{
        Message::ID::RtlPulsePacket,
        [this](Message* const p) {
            const auto message = static_cast<const RtlPulsePacketData*>(p);
            uint16_t min_pulses = (modulation_ == 0) ? 12 : 12;
            if (message == nullptr || message->num_pulses < min_pulses) {
                return;
            }
            this->on_packet(message);
        }};
};

}  // namespace ui

#endif /*__UI_RTL433_H__*/
