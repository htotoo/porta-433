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

#include "proc_rtl433.hpp"

#include "audio_dma.hpp"
#include "event_m4.hpp"
#include "portapack_shared_memory.hpp"

#include "dsp_fir_taps.hpp"

#include <algorithm>

RTL433Processor::RTL433Processor() {
    decim_0.configure(taps_200k_decim_0.taps);
    decim_1.configure(taps_200k_decim_1.taps);
    baseband_thread.start();
}

void RTL433Processor::execute(const buffer_c8_t& buffer) {
    if (!configured) {
        return;
    }

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
    feed_channel_stats(decim_1_out);

    for (size_t i = 0; i < decim_1_out.count; ++i) {
        const auto level = get_detection_level(decim_1_out.p[i]);
        bool raw_level = stable_level;

        if (modulation == Modulation::AM_OOK) {
            // rtl_433 style AM envelope thresholding with hysteresis.
            const uint32_t threshold = (ook_low_estimate + std::min(ook_high_estimate, ook_max_high_level)) / 2;
            const uint32_t hysteresis = std::max<uint32_t>(threshold / 8, 16);
            if (level > (threshold + hysteresis)) {
                raw_level = true;
            } else if (level < (threshold - hysteresis)) {
                raw_level = false;
            }
        } else {
            // SubTPMS-like FM slicing from discriminator sign with adaptive hysteresis.
            int32_t fm_hysteresis = fm_state.dev_lp >> 2;
            if (fm_hysteresis < 1024) {
                fm_hysteresis = 1024;
            }

            if (fm_state.discrim_lp > (fm_state.dc_lp + fm_hysteresis)) {
                raw_level = true;
            } else if (fm_state.discrim_lp < (fm_state.dc_lp - fm_hysteresis)) {
                raw_level = false;
            }
        }

        const bool high_level = apply_glitch_filter(raw_level);

        if (detect_package_from_level(level, high_level)) {
            if (pulse_data.num_pulses >= pd_min_pulses) {
                emit_pulse_package();
            }
            reset_pulse_detector();
        }
    }
}

void RTL433Processor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::SubGhzFPRxConfigure:
            configure(*reinterpret_cast<const SubGhzFPRxConfigureMessage*>(message));
            break;

        case Message::ID::AudioBeep:
            on_beep_message(*reinterpret_cast<const AudioBeepMessage*>(message));
            break;

        default:
            break;
    }
}

void RTL433Processor::configure(const SubGhzFPRxConfigureMessage& message) {
    modulation = (message.modulation == 1) ? Modulation::FM_FSK : Modulation::AM_OOK;

    if (message.sampling_rate != 0) {
        baseband_fs = message.sampling_rate;
    }
    baseband_thread.set_sampling_rate(baseband_fs);

    decimated_fs = static_cast<uint32_t>(baseband_fs / total_decimation);
    if (decimated_fs == 0) {
        decimated_fs = 1;
    }
    ns_per_decimated_sample = static_cast<uint32_t>(1000000000ULL / decimated_fs);

    // Keep FIRs simple and CPU-cheap for M4.
    decim_0.configure(taps_200k_decim_0.taps);
    decim_1.configure(taps_200k_decim_1.taps);

    reset_pulse_detector();
    configured = true;
}

void RTL433Processor::on_beep_message(const AudioBeepMessage& message) {
    audio::dma::beep_start(message.freq, message.sample_rate, message.duration_ms);
}

uint32_t RTL433Processor::get_detection_level(const complex16_t& sample) {
    const int16_t re = sample.real();
    const int16_t im = sample.imag();

    if (modulation == Modulation::AM_OOK) {
        const uint32_t mag_sq = static_cast<uint32_t>(re) * static_cast<uint32_t>(re) + static_cast<uint32_t>(im) * static_cast<uint32_t>(im);
        return mag_sq >> 10;
    }

    // Lightweight FM discriminator energy for package detection.
    const int16_t re_s = re >> 2;
    const int16_t im_s = im >> 2;
    const int32_t discrim = static_cast<int32_t>(im_s) * fm_state.last_re - static_cast<int32_t>(re_s) * fm_state.last_im;
    fm_state.last_re = re_s;
    fm_state.last_im = im_s;

    fm_state.discrim_lp += (discrim - fm_state.discrim_lp) >> 2;
    fm_state.dc_lp += (fm_state.discrim_lp - fm_state.dc_lp) >> 10;

    const int32_t centered = fm_state.discrim_lp - fm_state.dc_lp;
    const int32_t abs_centered = centered >= 0 ? centered : -centered;

    fm_state.dev_lp += (abs_centered - fm_state.dev_lp) >> 4;
    return static_cast<uint32_t>(fm_state.dev_lp);
}

bool RTL433Processor::detect_package_from_level(uint32_t level, bool level_is_high) {
    const uint32_t samples_per_ms = std::max<uint32_t>(decimated_fs / 1000, 1);
    bool end_of_package = false;

    // FM mode: explicit pulse/gap packaging path.
    if (modulation == Modulation::FM_FSK) {
        switch (ook_state) {
            case OOKState::Idle:
                if (level_is_high) {
                    pulse_data.clear();
                    pulse_data.sample_rate = decimated_fs;
                    pulse_length = 0;
                    max_pulse = 0;
                    ook_state = OOKState::Pulse;
                }
                break;

            case OOKState::Pulse:
                if (pulse_length < 0xFFFF) {
                    ++pulse_length;
                }

                if (!level_is_high) {
                    if (pulse_length < pd_min_pulse_samples) {
                        if (pulse_data.num_pulses <= 1) {
                            ook_state = OOKState::Idle;
                            pulse_length = 0;
                        } else {
                            end_of_package = true;
                            ook_state = OOKState::Gap;
                        }
                    } else {
                        if (pulse_data.num_pulses < PulseData::max_pulses) {
                            pulse_data.pulse[pulse_data.num_pulses] = pulse_length;
                        }
                        max_pulse = std::max(max_pulse, pulse_length);
                        pulse_length = 0;
                        ook_state = OOKState::GapStart;
                    }
                }
                break;

            case OOKState::GapStart:
                if (pulse_length < 0xFFFF) {
                    ++pulse_length;
                }

                if (level_is_high) {
                    if (pulse_data.num_pulses < PulseData::max_pulses) {
                        pulse_length = static_cast<uint16_t>(pulse_length + pulse_data.pulse[pulse_data.num_pulses]);
                    }
                    ook_state = OOKState::Pulse;
                } else if (pulse_length >= pd_min_pulse_samples) {
                    ook_state = OOKState::Gap;
                }
                break;

            case OOKState::Gap:
                if (pulse_length < 0xFFFF) {
                    ++pulse_length;
                }

                if (level_is_high) {
                    if (pulse_data.num_pulses < PulseData::max_pulses) {
                        pulse_data.gap[pulse_data.num_pulses] = pulse_length;
                        ++pulse_data.num_pulses;
                    }

                    if (pulse_data.num_pulses >= PulseData::max_pulses) {
                        end_of_package = true;
                    }

                    pulse_length = 0;
                    ook_state = OOKState::Pulse;
                }

                if (!end_of_package && (max_pulse > 0)) {
                    const bool gap_ratio_hit = pulse_length > (pd_max_gap_ratio * max_pulse) && pulse_length > (pd_min_gap_ms * samples_per_ms);
                    const bool max_gap_hit = pulse_length > (pd_max_gap_ms * samples_per_ms);
                    if (gap_ratio_hit || max_gap_hit) {
                        if (pulse_data.num_pulses < PulseData::max_pulses) {
                            pulse_data.gap[pulse_data.num_pulses] = pulse_length;
                            ++pulse_data.num_pulses;
                        }
                        end_of_package = true;
                    }
                }
                break;
        }

        return end_of_package;
    }

    switch (ook_state) {
        case OOKState::Idle:
            if (level_is_high && lead_in_counter > ook_est_low_ratio) {
                pulse_data.clear();
                pulse_data.sample_rate = decimated_fs;
                pulse_length = 0;
                max_pulse = 0;
                ook_state = OOKState::Pulse;
            } else {
                const int32_t low_delta = static_cast<int32_t>(level) - static_cast<int32_t>(ook_low_estimate);
                ook_low_estimate += low_delta / static_cast<int32_t>(ook_est_low_ratio);
                if (low_delta != 0) {
                    ook_low_estimate += (low_delta > 0) ? 1 : static_cast<uint32_t>(-1);
                }
                const uint32_t candidate_high = ook_low_estimate * 4;
                ook_high_estimate = std::max<uint32_t>(candidate_high, ook_min_high_level);
                if (lead_in_counter <= ook_est_low_ratio) {
                    ++lead_in_counter;
                }
            }
            break;

        case OOKState::Pulse:
            if (pulse_length < 0xFFFF) {
                ++pulse_length;
            }

            if (!level_is_high) {
                if (pulse_length < pd_min_pulse_samples) {
                    if (pulse_data.num_pulses <= 1) {
                        ook_state = OOKState::Idle;
                        pulse_length = 0;
                    } else {
                        end_of_package = true;
                        ook_state = OOKState::Gap;
                    }
                } else {
                    if (pulse_data.num_pulses < PulseData::max_pulses) {
                        pulse_data.pulse[pulse_data.num_pulses] = pulse_length;
                    }
                    max_pulse = std::max(max_pulse, pulse_length);
                    pulse_length = 0;
                    ook_state = OOKState::GapStart;
                }
            } else {
                ook_high_estimate += (level / ook_est_high_ratio) - (ook_high_estimate / ook_est_high_ratio);
                ook_high_estimate = std::max<uint32_t>(ook_high_estimate, ook_min_high_level);
            }
            break;

        case OOKState::GapStart:
            if (pulse_length < 0xFFFF) {
                ++pulse_length;
            }

            if (level_is_high) {
                if (pulse_data.num_pulses < PulseData::max_pulses) {
                    pulse_length = static_cast<uint16_t>(pulse_length + pulse_data.pulse[pulse_data.num_pulses]);
                }
                ook_state = OOKState::Pulse;
            } else if (pulse_length >= pd_min_pulse_samples) {
                ook_state = OOKState::Gap;
            }
            break;

        case OOKState::Gap:
            if (pulse_length < 0xFFFF) {
                ++pulse_length;
            }

            if (level_is_high) {
                if (pulse_data.num_pulses < PulseData::max_pulses) {
                    pulse_data.gap[pulse_data.num_pulses] = pulse_length;
                    ++pulse_data.num_pulses;
                }

                if (pulse_data.num_pulses >= PulseData::max_pulses) {
                    end_of_package = true;
                }

                pulse_length = 0;
                ook_state = OOKState::Pulse;
            }

            if (!end_of_package && (max_pulse > 0)) {
                const bool gap_ratio_hit = pulse_length > (pd_max_gap_ratio * max_pulse) && pulse_length > (pd_min_gap_ms * samples_per_ms);
                const bool max_gap_hit = pulse_length > (pd_max_gap_ms * samples_per_ms);
                if (gap_ratio_hit || max_gap_hit) {
                    if (pulse_data.num_pulses < PulseData::max_pulses) {
                        pulse_data.gap[pulse_data.num_pulses] = pulse_length;
                        ++pulse_data.num_pulses;
                    }
                    end_of_package = true;
                }
            }
            break;
    }

    return end_of_package;
}

bool RTL433Processor::apply_glitch_filter(bool raw_level) {
    if (raw_level == stable_level) {
        glitch_duration = 0;
        return stable_level;
    }

    glitch_duration += ns_per_decimated_sample;
    if (glitch_duration >= 15000) {
        stable_level = raw_level;
        glitch_duration = 0;
    }
    return stable_level;
}

void RTL433Processor::emit_pulse_package() {
    if (pulse_data.num_pulses == 0) {
        return;
    }

    RtlPulsePacketData message{};
    message.num_pulses = pulse_data.num_pulses;
    message.sample_rate = pulse_data.sample_rate;
    message.ook_low_estimate = ook_low_estimate;
    message.ook_high_estimate = ook_high_estimate;

    for (uint16_t i = 0; i < pulse_data.num_pulses && i < RtlPulsePacketData::max_pulses; ++i) {
        message.pulse[i] = pulse_data.pulse[i];
        message.gap[i] = pulse_data.gap[i];
    }

    shared_memory.application_queue.push(message);
}

void RTL433Processor::reset_pulse_detector() {
    pulse_data.clear();
    pulse_data.sample_rate = decimated_fs;
    pulse_data.ook_low_estimate = ook_low_estimate;
    pulse_data.ook_high_estimate = ook_high_estimate;

    ook_state = OOKState::Idle;
    pulse_length = 0;
    max_pulse = 0;
    lead_in_counter = 0;

    if (ook_low_estimate == 0) {
        ook_low_estimate = 10;
    }
    if (ook_high_estimate < ook_min_high_level) {
        ook_high_estimate = ook_min_high_level;
    }

    glitch_duration = 0;
    stable_level = false;

    fm_state = {};
}

int main() {
    audio::dma::init_audio_out();
    EventDispatcher event_dispatcher{std::make_unique<RTL433Processor>()};
    event_dispatcher.run();
    return 0;
}
