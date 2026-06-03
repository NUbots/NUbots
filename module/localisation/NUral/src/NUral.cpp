#include "NUral.hpp"

#include "extension/Configuration.hpp"

#include "message/platform/RawSensors.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::platform::RawSensors;

    NUral::NUral(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUral.yaml").then([this](const Configuration& config) {
            // Use configuration here from file NUral.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            this->cfg.model_path = config["model_path"].as<std::string>();

            // Compile the model and create inference request object
            try {
                compiled_model = core.compile_model(cfg.model_path, "CPU");
                infer_request  = compiled_model.create_infer_request();
                log<INFO>("Model loaded successfully");
            }
            catch (const std::exception& e) {
                log./ b<ERROR>("Failed to load NUral model: ", e.what());
                throw;
            }
        });

        on<Trigger<RawSensors>>().then([this](const RawSensors& raw_sensors) {
            // Pass the raw sensor data through the NUral model to get a localisation estimate, then log the result.

            // input tensor should be [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
            ov::Tensor input_tensor = infer_request.get_input_tensor();
            float* input_data       = input_tensor.data<float>();
            input_data[0]           = raw_sensors.accelerometer.x();
            input_data[1]           = raw_sensors.accelerometer.y();
            input_data[2]           = raw_sensors.accelerometer.z();
            input_data[3]           = raw_sensors.gyroscope.x();
            input_data[4]           = raw_sensors.gyroscope.y();
            input_data[5]           = raw_sensors.gyroscope.z();

            // Perform inference
            try {
                infer_request.infer();
            }
            catch (const std::exception& e) {
                log<ERROR>("Inference failed: ", e.what());
                return;
            }

            // output will be of the form [predicted_x, predicted_y, predicted_z, predicted_rot_x, predicted_rot_y,
            // predicted_rot_z]
            float* output_data = infer_request.get_output_tensor(0).data<float>();
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
