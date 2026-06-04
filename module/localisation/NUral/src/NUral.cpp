#include "NUral.hpp"

#include <filesystem>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/NUral.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::input::Sensors;

    float filtered_data[6] = {0};  // to store the filtered localisation estimate

    double cum_x = 0.0, cum_y = 0.0, cum_z = 0.0;     // cumulative sum of accelerometer readings
    double cum_gx = 0.0, cum_gy = 0.0, cum_gz = 0.0;  // cumulative sum of gyroscope readings

    NUral::NUral(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUral.yaml").then([this](const Configuration& config) {
            // Use configuration here from file NUral.yaml
            this->log_level                 = config["log_level"].as<NUClear::LogLevel>();
            std::string model_path          = config["model_path"].as<std::string>();
            this->cfg.translation_threshold = config["translation_threshold"].as<double>();
            this->cfg.rotation_threshold    = config["rotation_threshold"].as<double>();
            // Compile the model and create inference request object
            try {
                log<INFO>("Loading NUral model from: ", model_path);
                compiled_model = core.compile_model(model_path, "CPU");
                infer_request  = compiled_model.create_infer_request();
                model_loaded   = true;
                log<INFO>("Model loaded successfully");
            }
            catch (const std::exception& e) {
                log<ERROR>("Failed to load NUral model: ", e.what());
                throw;
            }
        });

        on<Trigger<Sensors>, Sync<message::localisation::NUral>>().then([this](const Sensors& sensors) {
            // Pass the raw sensor data through the NUral model to get a localisation estimate, then log the result.
            if (!model_loaded) {
                log<ERROR>("NUral model is not loaded, skipping inference");
                return;
            }

            // just print all the raw sensor data
            log<DEBUG>("Raw sensor data - Accel: ({}, {}, {}), Gyro: ({}, {}, {})",
                       sensors.accelerometer.x(),
                       sensors.accelerometer.y(),
                       sensors.accelerometer.z(),
                       sensors.gyroscope.x(),
                       sensors.gyroscope.y(),
                       sensors.gyroscope.z());

            // input tensor should be [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
            auto input_port = compiled_model.input();
            ov::Tensor input_tensor(input_port.get_element_type(), ov::Shape{1, 6});
            float* input_data = input_tensor.data<float>();
            input_data[0]     = sensors.accelerometer.x();
            input_data[1]     = sensors.accelerometer.y();
            input_data[2]     = sensors.accelerometer.z();
            input_data[3]     = sensors.gyroscope.x();
            input_data[4]     = sensors.gyroscope.y();
            input_data[5]     = sensors.gyroscope.z();
            infer_request.set_input_tensor(input_tensor);

            // Perform inference
            try {
                infer_request.infer();
            }
            catch (const std::exception& e) {
                log<ERROR>("Inference failed: ", e.what());
                return;
            }

            auto output_tensor = infer_request.get_output_tensor(0);
            log<DEBUG>("NUral output tensor shape: ", output_tensor.get_shape(), ", size: ", output_tensor.get_size());

            if (output_tensor.get_size() < 6) {
                log<ERROR>("NUral model output is too small to read localisation estimate: ", output_tensor.get_size());
                return;
            }

            const float* output_data = output_tensor.data<float>();
            log<DEBUG>("Localisation estimate: ({}, {}, {}), Rotation: ({}, {}, {})",
                       output_data[0],
                       output_data[1],
                       output_data[2],
                       output_data[3],
                       output_data[4],
                       output_data[5]);

            // Simple threshold filter
            for (int i = 0; i < 3; i++) {
                if (std::abs(output_data[i]) < cfg.translation_threshold) {
                    filtered_data[i] = 0.0f;
                }
                else {
                    filtered_data[i] = output_data[i];
                }
            }
            for (int i = 3; i < 6; i++) {
                if (std::abs(output_data[i]) < cfg.rotation_threshold) {
                    filtered_data[i] = 0.0f;
                }
                else {
                    filtered_data[i] = output_data[i];
                }
            }
            log<DEBUG>("Filtered localisation estimate: ({}, {}, {}), Rotation: ({}, {}, {})",
                       filtered_data[0],
                       filtered_data[1],
                       filtered_data[2],
                       filtered_data[3],
                       filtered_data[4],
                       filtered_data[5]);

            cum_x += filtered_data[0];
            cum_y += filtered_data[1];
            cum_z += filtered_data[2];

            cum_gx += filtered_data[3];
            cum_gy += filtered_data[4];
            cum_gz += filtered_data[5];

            log<DEBUG>("Cumulative sensor data - Accel: ({}, {}, {}), Gyro: ({}, {}, {})",
                       cum_x,
                       cum_y,
                       cum_z,
                       cum_gx,
                       cum_gy,
                       cum_gz);

            // Publish the filtered localisation estimate
            /*
            auto msg             = std::make_unique<message::localisation::NUral>();
            msg->estimated_x     = filtered_data[0];
            msg->estimated_y     = filtered_data[1];
            msg->estimated_z     = filtered_data[2];
            msg->estimated_roll  = filtered_data[3];
            msg->estimated_pitch = filtered_data[4];
            msg->estimated_yaw   = filtered_data[5];
            emit(std::move(msg));
            */

            // Dummy data - just all zeros
            auto msg             = std::make_unique<message::localisation::NUral>();
            msg->estimated_x     = 0.0f;
            msg->estimated_y     = 0.0f;
            msg->estimated_z     = 0.0f;
            msg->estimated_roll  = 0.0f;
            msg->estimated_pitch = 0.0f;
            msg->estimated_yaw   = 0.0f;
            emit(std::move(msg));
        });
    }

}  // namespace module::localisation
