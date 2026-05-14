/*
 * Copyright (C) 2026 HTotoo
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

    if (modulation == Modulation::AM_OOK) {
        for (size_t i = 0; i < decim_1_out.count; ++i) {
            const int32_t re = decim_1_out.p[i].real();
            const int32_t im = decim_1_out.p[i].imag();
            const uint32_t level = static_cast<uint32_t>(re * re + im * im) >> 10;

            const uint32_t high_est = (ook_high_estimate < ook_max_high_level) ? ook_high_estimate : ook_max_high_level;
            const uint32_t threshold = (ook_low_estimate + high_est) >> 1;
            const uint32_t thresh_div_8 = threshold >> 3;
            const uint32_t hysteresis = (thresh_div_8 > 16) ? thresh_div_8 : 16;

            bool raw_level = stable_level;
            if (level > (threshold + hysteresis)) {
                raw_level = true;
            } else if (level < (threshold - hysteresis)) {
                raw_level = false;
            }

            if (raw_level == stable_level) {
                glitch_duration = 0;
            } else {
                if (glitch_duration < 0xFFFFFFFF) ++glitch_duration;
                if (glitch_duration >= glitch_min_samples) {
                    stable_level = raw_level;
                    glitch_duration = 0;
                }
            }
            const bool high_level = stable_level;

            if (!high_level && ook_state == OOKState::Idle) {
                if ((i & 0x0F) == 0) {
                    int32_t error = static_cast<int32_t>(level) - static_cast<int32_t>(ook_low_estimate);
                    int32_t step = error >> 10;
                    if (step == 0 && error != 0) {
                        step = (error > 0) ? 1 : -1;
                    }
                    int32_t new_est = static_cast<int32_t>(ook_low_estimate) + step;
                    ook_low_estimate = (new_est < 0) ? 0 : static_cast<uint32_t>(new_est);

                    uint32_t candidate_high = ook_low_estimate * 4;
                    ook_high_estimate = (candidate_high > ook_min_high_level) ? candidate_high : ook_min_high_level;

                    if (lead_in_counter <= ook_est_low_ratio) ++lead_in_counter;
                }
                continue;  // Ugorjuk át a drága detect állapotgépet
            }

            if (detect_package_from_level_am(level, high_level)) {
                if (pulse_data.num_pulses >= pd_min_pulses) {
                    emit_pulse_package();
                }
                reset_pulse_detector();
            }
        }
        return;
    }

    // FM
    for (size_t i = 0; i < decim_1_out.count; ++i) {
        const int32_t re = decim_1_out.p[i].real();
        const int32_t im = decim_1_out.p[i].imag();
        const int32_t re_s = re >> 2;
        const int32_t im_s = im >> 2;
        const int32_t discrim = (im_s * fm_state.last_re) - (re_s * fm_state.last_im);
        fm_state.last_re = re_s;
        fm_state.last_im = im_s;

        fm_state.discrim_lp += (discrim - fm_state.discrim_lp) >> 2;
        fm_state.dc_lp += (fm_state.discrim_lp - fm_state.dc_lp) >> 10;

        const int32_t centered = fm_state.discrim_lp - fm_state.dc_lp;
        const int32_t abs_centered = (centered >= 0) ? centered : -centered;

        fm_state.dev_lp += (abs_centered - fm_state.dev_lp) >> 4;
        const uint32_t fm_level = static_cast<uint32_t>(fm_state.dev_lp);

        if (fm_level < fm_min_dev_for_detect) {
            if (!stable_level && ook_state == OOKState::Idle) {
                glitch_duration = 0;
                continue;  // FM FAST PATH
            }

            if (false == stable_level) {
                glitch_duration = 0;
            } else {
                if (glitch_duration < 0xFFFFFFFF) ++glitch_duration;
                if (glitch_duration >= glitch_min_samples) {
                    stable_level = false;
                    glitch_duration = 0;
                }
            }

            if (detect_package_from_level_fm(stable_level)) {
                if (pulse_data.num_pulses >= pd_min_pulses) {
                    emit_pulse_package();
                }
                reset_pulse_detector();
            }
            continue;
        }

        int32_t fm_hysteresis = fm_state.dev_lp >> 2;
        if (fm_hysteresis < 1024) fm_hysteresis = 1024;

        bool raw_level = stable_level;
        if (fm_state.discrim_lp > (fm_state.dc_lp + fm_hysteresis)) {
            raw_level = true;
        } else if (fm_state.discrim_lp < (fm_state.dc_lp - fm_hysteresis)) {
            raw_level = false;
        }

        if (raw_level == stable_level) {
            glitch_duration = 0;
        } else {
            if (glitch_duration < 0xFFFFFFFF) ++glitch_duration;
            if (glitch_duration >= glitch_min_samples) {
                stable_level = raw_level;
                glitch_duration = 0;
            }
        }

        if (detect_package_from_level_fm(stable_level)) {
            if (pulse_data.num_pulses >= pd_min_pulses) emit_pulse_package();
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

    uint32_t required_samples = (15000U + ns_per_decimated_sample - 1) / ns_per_decimated_sample;
    glitch_min_samples = (required_samples > 1) ? required_samples : 1;

    uint32_t dec_fs_div_1000 = decimated_fs / 1000;
    const uint32_t samples_per_ms = (dec_fs_div_1000 > 1) ? dec_fs_div_1000 : 1;

    min_gap_samples = pd_min_gap_ms * samples_per_ms;
    max_gap_samples = pd_max_gap_ms * samples_per_ms;

    decim_0.configure(taps_200k_decim_0.taps);
    decim_1.configure(taps_200k_decim_1.taps);

    reset_pulse_detector();
    configured = true;
}

void RTL433Processor::on_beep_message(const AudioBeepMessage& message) {
    audio::dma::beep_start(message.freq, message.sample_rate, message.duration_ms);
}

inline bool RTL433Processor::detect_package_from_level_fm(bool level_is_high) {
    bool end_of_package = false;

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
            if (pulse_length < 0xFFFF) ++pulse_length;

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
                    if (pulse_length > max_pulse) max_pulse = pulse_length;
                    pulse_length = 0;
                    ook_state = OOKState::GapStart;
                }
            }
            break;

        case OOKState::GapStart:
            if (pulse_length < 0xFFFF) ++pulse_length;

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
            if (pulse_length < 0xFFFF) ++pulse_length;

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
                const bool gap_ratio_hit = pulse_length > (pd_max_gap_ratio * max_pulse) && pulse_length > min_gap_samples;
                const bool max_gap_hit = pulse_length > max_gap_samples;
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

inline bool RTL433Processor::detect_package_from_level_am(uint32_t level, bool level_is_high) {
    bool end_of_package = false;

    switch (ook_state) {
        case OOKState::Idle:
            if (level_is_high && lead_in_counter > ook_est_low_ratio) {
                pulse_data.clear();
                pulse_data.sample_rate = decimated_fs;
                pulse_length = 0;
                max_pulse = 0;
                ook_state = OOKState::Pulse;
            } else {
                int32_t error = static_cast<int32_t>(level) - static_cast<int32_t>(ook_low_estimate);
                int32_t step = error >> 10;
                if (step == 0 && error != 0) step = (error > 0) ? 1 : -1;

                int32_t new_est = static_cast<int32_t>(ook_low_estimate) + step;
                ook_low_estimate = (new_est < 0) ? 0 : static_cast<uint32_t>(new_est);

                const uint32_t candidate_high = ook_low_estimate * 4;
                ook_high_estimate = (candidate_high > ook_min_high_level) ? candidate_high : ook_min_high_level;
                if (lead_in_counter <= ook_est_low_ratio) ++lead_in_counter;
            }
            break;

        case OOKState::Pulse:
            if (pulse_length < 0xFFFF) ++pulse_length;

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
                    if (pulse_length > max_pulse) max_pulse = pulse_length;
                    pulse_length = 0;
                    ook_state = OOKState::GapStart;
                }
            } else {
                ook_high_estimate += (level >> 6) - (ook_high_estimate >> 6);
                if (ook_high_estimate < ook_min_high_level) ook_high_estimate = ook_min_high_level;
            }
            break;

        case OOKState::GapStart:
            if (pulse_length < 0xFFFF) ++pulse_length;

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
            if (pulse_length < 0xFFFF) ++pulse_length;

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
                const bool gap_ratio_hit = pulse_length > (pd_max_gap_ratio * max_pulse) && pulse_length > min_gap_samples;
                const bool max_gap_hit = pulse_length > max_gap_samples;
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

inline void RTL433Processor::emit_pulse_package() {
    if (pulse_data.num_pulses == 0) return;
    if (!shared_memory.application_queue.is_empty()) return;

    uint32_t sig = 0x9e3779b9u ^ static_cast<uint32_t>(pulse_data.num_pulses);
    uint16_t sig_len = (pulse_data.num_pulses < 10) ? pulse_data.num_pulses : 10;

    for (uint16_t i = 0; i < sig_len; ++i) {
        sig ^= static_cast<uint32_t>(pulse_data.pulse[i]) + 0x85ebca6bu + (sig << 6) + (sig >> 2);
        sig ^= static_cast<uint32_t>(pulse_data.gap[i]) + 0xc2b2ae35u + (sig << 6) + (sig >> 2);
    }

    if (sig == last_tx_signature_ && pulse_data.num_pulses == last_tx_pulses_) {
        if (repeated_tx_count_ < 3) {
            ++repeated_tx_count_;
            return;
        }
        repeated_tx_count_ = 0;
    } else {
        last_tx_signature_ = sig;
        last_tx_pulses_ = pulse_data.num_pulses;
        repeated_tx_count_ = 0;
    }

    tx_packet_.num_pulses = pulse_data.num_pulses;
    tx_packet_.sample_rate = pulse_data.sample_rate;
    tx_packet_.ook_low_estimate = ook_low_estimate;
    tx_packet_.ook_high_estimate = ook_high_estimate;

    for (uint16_t i = 0; i < pulse_data.num_pulses && i < RtlPulsePacketData::max_pulses; ++i) {
        tx_packet_.pulse[i] = pulse_data.pulse[i];
        tx_packet_.gap[i] = pulse_data.gap[i];
    }

    shared_memory.application_queue.push(tx_packet_);
}

void RTL433Processor::reset_pulse_detector() {
    pulse_data.num_pulses = 0;
    pulse_data.sample_rate = decimated_fs;
    pulse_data.ook_low_estimate = ook_low_estimate;
    pulse_data.ook_high_estimate = ook_high_estimate;

    ook_state = OOKState::Idle;
    pulse_length = 0;
    max_pulse = 0;
    lead_in_counter = 0;

    if (ook_low_estimate == 0) ook_low_estimate = 10;
    if (ook_high_estimate < ook_min_high_level) ook_high_estimate = ook_min_high_level;

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
