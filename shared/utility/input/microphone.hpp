/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_INPUT_MICROPHONE_HPP
#define UTILITY_INPUT_MICROPHONE_HPP

#include <alsa/asoundlib.h>
#include <fstream>
#include <lame/lame.h>

namespace utility::input {

    /**
     * @brief Record audio using microphone for set duration
     * @param filename name of the file to save the audio to
     * @param duration_seconds duration of the audio recording in seconds
     */
    void record_audio(const std::string& filename, int duration_seconds) {
        // ALSA parameters
        snd_pcm_t* handle;
        snd_pcm_hw_params_t* params;
        unsigned int sample_rate = 44100;
        int dir;
        snd_pcm_uframes_t frames = 32;  // frames per period
        char* buffer;
        int channels = 2;  // stereo

        // Open PCM device for recording (capture)
        if (snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0) < 0) {
            throw std::runtime_error("Failed to open PCM device");
        }

        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(handle, params);

        // Set parameters for ALSA
        snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(handle, params, channels);
        snd_pcm_hw_params_set_rate_near(handle, params, &sample_rate, &dir);
        snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

        snd_pcm_hw_params(handle, params);

        buffer = (char*) malloc(frames * channels * 2);  // 2 bytes per sample for SND_PCM_FORMAT_S16_LE

        std::ofstream out_file(filename, std::ios::binary);

        int num_periods_per_second = sample_rate / frames;
        int total_periods          = duration_seconds * num_periods_per_second;

        for (int i = 0; i < total_periods; ++i) {
            snd_pcm_readi(handle, buffer, frames);
            out_file.write(buffer, frames * channels * 2);
        }

        // Cleanup
        out_file.close();
        free(buffer);
        snd_pcm_close(handle);
    }

    /**
     * @brief Convert raw audio to mp3
     * @param input_file name of the file to read raw audio from
     * @param output_file name of the file to save the mp3 to
     */
    void raw_to_mp3(const std::string& input_file, const std::string& output_file) {
        // LAME parameters
        int channels             = 2;
        unsigned int sample_rate = 44100;
        snd_pcm_uframes_t frames = 32;

        lame_t lame = lame_init();
        lame_set_num_channels(lame, channels);
        lame_set_in_samplerate(lame, sample_rate);
        lame_set_brate(lame, 128);
        lame_set_mode(lame, STEREO);
        lame_set_quality(lame, 2);
        lame_init_params(lame);

        std::vector<short> buffer(frames * channels);
        std::vector<unsigned char> mp3_buffer(1.25 * frames * channels + 7200);

        std::ifstream in_file(input_file, std::ios::binary);
        std::ofstream out_file(output_file, std::ios::binary);

        while (!in_file.eof()) {
            in_file.read((char*) buffer.data(), frames * channels * 2);
            int bytes_read = in_file.gcount();

            if (bytes_read <= 0) {
                NUClear::log<NUClear::ERROR>("Failed to read from input file");
                break;
            }

            int write_size = lame_encode_buffer_interleaved(lame,
                                                            buffer.data(),
                                                            bytes_read / (2 * channels),
                                                            mp3_buffer.data(),
                                                            mp3_buffer.size());
            out_file.write((const char*) mp3_buffer.data(), write_size);
        }

        int flush_size = lame_encode_flush(lame, mp3_buffer.data(), mp3_buffer.size());
        out_file.write((const char*) mp3_buffer.data(), flush_size);

        in_file.close();
        out_file.close();
        lame_close(lame);
    }

}  // namespace utility::input

#endif  // UTILITY_INPUT_MICROPHONE_HPP
