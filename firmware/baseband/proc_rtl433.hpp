/*
 * Copyright (C) 2026
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef __PROC_RTL_H__
#define __PROC_RTL_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"

#include "dsp_decimate.hpp"
#include "message.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

class RTL433Processor : public BasebandProcessor {
   public:
    RTL433Processor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    enum class Modulation : uint8_t {
        AM_OOK = 0,
        FM_FSK = 1,
    };

    enum class OOKState : uint8_t {
        Idle = 0,
        Pulse,
        GapStart,
        Gap,
    };

    struct PulseData {
        static constexpr size_t max_pulses = 256;

        uint16_t pulse[max_pulses]{};
        uint16_t gap[max_pulses]{};
        uint16_t num_pulses{0};

        uint32_t sample_rate{0};
        uint32_t ook_low_estimate{0};
        uint32_t ook_high_estimate{0};

        void clear() {
            num_pulses = 0;
        }
    };

    struct FmState {
        int16_t last_re{0};
        int16_t last_im{0};
        int32_t discrim_lp{0};
        int32_t dc_lp{0};
        int32_t dev_lp{0};
    };

    static constexpr uint32_t baseband_fs_default = 2457600;
    static constexpr uint8_t total_decimation = 8;

    static constexpr uint16_t pd_min_pulses = 16;
    static constexpr uint16_t pd_min_pulse_samples = 3;
    static constexpr uint16_t pd_min_gap_ms = 10;
    static constexpr uint16_t pd_max_gap_ms = 100;
    static constexpr uint8_t pd_max_gap_ratio = 10;

    static constexpr uint32_t ook_est_high_ratio = 64;
    static constexpr uint32_t ook_est_low_ratio = 1024;
    static constexpr uint32_t ook_min_high_level = 1000;
    static constexpr uint32_t ook_max_high_level = 32768;

    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{dst.data(), dst.size()};

    dsp::decimate::FIRC8xR16x24FS4Decim4 decim_0{};
    dsp::decimate::FIRC16xR16x16Decim2 decim_1{};

    size_t baseband_fs{baseband_fs_default};
    uint32_t decimated_fs{baseband_fs_default / total_decimation};
    uint32_t ns_per_decimated_sample{0};

    bool configured{false};
    Modulation modulation{Modulation::AM_OOK};
    uint32_t glitch_duration{0};
    bool stable_level{false};

    OOKState ook_state{OOKState::Idle};
    uint16_t pulse_length{0};
    uint16_t max_pulse{0};
    uint32_t lead_in_counter{0};
    uint32_t ook_low_estimate{0};
    uint32_t ook_high_estimate{ook_min_high_level};

    PulseData pulse_data{};
    FmState fm_state{};

    void configure(const SubGhzFPRxConfigureMessage& message);
    void on_beep_message(const AudioBeepMessage& message);

    uint32_t get_detection_level(const complex16_t& sample);
    bool detect_package_from_level(uint32_t level, bool level_is_high);
    bool apply_glitch_filter(bool raw_level);
    void emit_pulse_package();
    void reset_pulse_detector();

    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_RTL_H__*/
