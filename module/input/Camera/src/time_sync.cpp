/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "time_sync.hpp"

extern "C" {
#include <aravis-0.8/arv.h>
}

#include "aravis_wrap.hpp"

namespace module::input {

    CameraContext::TimeCorrection sync_clocks(ArvDevice* device) {
        using namespace std::chrono;
        struct Sample {
            int64_t local;
            int64_t cam;
            bool valid;
        };

        // This is what we return if we can't work out the timestamp offset now
        // Note that all these times are measured in nanoseconds
        CameraContext::TimeCorrection output{};
        output.live   = true;
        output.offset = 0;
        output.kf.p   = 10e6 * 10e6;    // Initial stddev of about 10 milliseconds
        output.kf.r   = 500e3 * 500e3;  // Standard deviation of about 500 microseconds
        output.kf.q   = 1e3 * 1e3;      // 1 microsecond standard deviation of process noise

        // The samples of the two clocks
        std::array<Sample, 20> samples{};
        samples.fill(Sample{0, 0, false});

        // Commands used to latch the timestamp and read the timestamp
        std::string latch_command;
        std::string read_command;

        // FLIR like control
        if ((arv_device_get_feature(device, "TimestampLatch") != nullptr)
            && (arv_device_get_feature(device, "TimestampLatchValue") != nullptr)) {
            latch_command = "TimestampLatch";
            read_command  = "TimestampLatchValue";
        }
        // Genicam like control
        else if ((arv_device_get_feature(device, "GevTimestampControlLatch") != nullptr)
                 && (arv_device_get_feature(device, "GevTimestampValue") != nullptr)) {
            latch_command = "GevTimestampControlLatch";
            read_command  = "GevTimestampValue";
        }

        for (auto& s : samples) {
            // Get the time before and after sending the command. We know neither are correct and it's likely
            // somewhere in the middle that the actual latching took place. Therefore take the average of the two to
            // get closer.
            try {

                NUClear::clock::time_point t1 = NUClear::clock::now();
                arv::device_execute_command(device, latch_command.c_str());
                NUClear::clock::time_point t2 = NUClear::clock::now();
                s.local = duration_cast<nanoseconds>((t1.time_since_epoch() + t2.time_since_epoch()) / 2).count();

                // Try to read the time we just latched off the camera
                s.cam = arv::device_get_integer_feature_value(device, read_command.c_str());
            }
            catch (const std::runtime_error&) {
                s.valid = false;
            }
        }

        // Sort all the invalid entries to the end
        auto* end = std::stable_partition(samples.begin(), samples.end(), [](const Sample& s) { return s.valid; });

        // Use the samples to work out the offset between our clock and the cameras clock and if this sync is valid
        int n_samples = 0;  // The number of samples that were valid
        int64_t delta = 0;  // The difference between the two clocks

        // There were no valid samples, return 0 as the offset
        // Getting 0 as an offset is almost impossible at nanosecond precision and if it ever did happen
        // Then just using our own timestamp instead of the one from the camera is going to be pretty close
        if (samples.begin() == end) {
            return output;
        }

        // The total amount of time taken during the test on the local and remote systems for checking if it worked
        // These numbers should be the same to within a fairly tight tolerance
        int64_t local_total = std::prev(end, 1)->local - samples.begin()->local;
        int64_t cam_total   = std::prev(end, 1)->cam - samples.begin()->cam;

        // Get the average of the clock offsets
        for (const auto& s : samples) {
            if (s.valid) {
                ++n_samples;
                delta += s.local - s.cam;
            }
        }
        // Check if the total time that elapsed on the device vs the local computer is reasonable
        double accuracy = double(std::min(local_total, cam_total)) / double(std::max(local_total, cam_total));
        if (accuracy > 0.95) {
            output.live                       = false;
            output.offset                     = delta / n_samples;
            output.drift.cam_at_calibration   = samples.front().cam;
            output.drift.local_at_calibration = samples.front().local;
            // Set a value to recalibrate if our error accumulates too much
            output.drift.max_clock_drift = 0;
            output.drift.over_time_count = 0;
            return output;
        }
        return output;
    }
}  // namespace module::input
