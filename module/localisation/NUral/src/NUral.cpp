#include "NUral.hpp"

#include <filesystem>

#include "extension/Configuration.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::platform::RawSensors;
    using message::platform::webots::AccelerometerMeasurement;
    using message::platform::webots::GyroMeasurement;

    NUral::NUral(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUral.yaml").then([this](const Configuration& config) {
            // Use configuration here from file NUral.yaml
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            std::string model_path = config["model_path"].as<std::string>();
            // Compile the model and create inference request object
            try {
                log<INFO>("Loading NUral model from: ", model_path);
                compiled_model = core.compile_model(model_path, "CPU");
                infer_request  = compiled_model.create_infer_request();
                log<INFO>("Model loaded successfully");
            }
            catch (const std::exception& e) {
                log<ERROR>("Failed to load NUral model: ", e.what());
                throw;
            }
        });

        on<Trigger<RawSensors>, Sync<NUral>>().then([this](const RawSensors& raw_sensors) {
            // Pass the raw sensor data through the NUral model to get a localisation estimate, then log the result.
            // just print all the raw sensor data
            log<DEBUG>("Raw sensor data - Accel: ({}, {}, {}), Gyro: ({}, {}, {})",
                       raw_sensors.accelerometer.x(),
                       raw_sensors.accelerometer.y(),
                       raw_sensors.accelerometer.z(),
                       raw_sensors.gyroscope.x(),
                       raw_sensors.gyroscope.y(),
                       raw_sensors.gyroscope.z());

            // input tensor should be [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
            auto input_port = compiled_model.input();
            ov::Tensor input_tensor(input_port.get_element_type(), ov::Shape{1, 6});
            float* input_data = input_tensor.data<float>();
            input_data[0]     = raw_sensors.accelerometer.x();
            input_data[1]     = raw_sensors.accelerometer.y();
            input_data[2]     = raw_sensors.accelerometer.z();
            input_data[3]     = raw_sensors.gyroscope.x();
            input_data[4]     = raw_sensors.gyroscope.y();
            input_data[5]     = raw_sensors.gyroscope.z();
            infer_request.set_input_tensor(input_tensor);

            // Perform inference
            try {
                infer_request.infer();
                log<INFO>("Inference successful");
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
            log<INFO>("Localisation estimate: ({}, {}, {}), Rotation: ({}, {}, {})",
                      output_data[0],
                      output_data[1],
                      output_data[2],
                      output_data[3],
                      output_data[4],
                      output_data[5]);
        });
    }

}  // namespace module::localisation
