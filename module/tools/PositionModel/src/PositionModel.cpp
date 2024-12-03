#include "PositionModel.hpp"

#include "extension/Configuration.hpp"
#include "message/platform/RawSensors.hpp"
#include <filesystem>
#include <string>
#include <sstream>
#include <fmt/format.h>

namespace module::tools {

using extension::Configuration;
using message::platform::RawSensors;

namespace fs=std::filesystem;

PositionModel::PositionModel(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("PositionModel.yaml").then([this](const Configuration& config) {
        // Use configuration here from file PositionModel.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
        // auto file_name = config["file_name"].as<std::string>();
        csv_ofs.open(fmt::format("recordings/{}", config["file_name"].as<std::string>()));

        // Define CSV column headers
        std::stringstream column_names;
        column_names << "timestamp";

        // Add servo field names to the CSV headers
        std::vector<std::string> servo_names = {
            "r_shoulder_pitch", "l_shoulder_pitch", "r_shoulder_roll", "l_shoulder_roll",
            "r_elbow", "l_elbow", "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
            "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee", "r_ankle_pitch",
            "l_ankle_pitch", "r_ankle_roll", "l_ankle_roll", "head_pan", "head_tilt"
        };

        for (const auto& name : servo_names) {
            column_names << "," << name;
        }

        // Finalize and write headers to the CSV file
        column_names << "\n";
        csv_ofs << column_names.str();
    });


    on<Trigger<RawSensors>, Sync<PositionModel>>().then([this](const RawSensors& raw_sensors){
        auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        raw_sensors.timestamp.time_since_epoch())
                        .count();
        csv_ofs << timestamp_ns;

        // Write sensor data to the CSV file, properly formatted
        csv_ofs << raw_sensors.servo.r_shoulder_pitch.present_position << ",";
        csv_ofs << raw_sensors.servo.l_shoulder_pitch.present_position << ",";
        csv_ofs << raw_sensors.servo.r_shoulder_roll.present_position << ",";
        csv_ofs << raw_sensors.servo.l_shoulder_roll.present_position << ",";
        csv_ofs << raw_sensors.servo.r_elbow.present_position << ",";
        csv_ofs << raw_sensors.servo.l_elbow.present_position << ",";
        csv_ofs << raw_sensors.servo.r_hip_yaw.present_position << ",";
        csv_ofs << raw_sensors.servo.l_hip_yaw.present_position << ",";
        csv_ofs << raw_sensors.servo.r_hip_roll.present_position << ",";
        csv_ofs << raw_sensors.servo.l_hip_roll.present_position << ",";
        csv_ofs << raw_sensors.servo.r_hip_pitch.present_position << ",";
        csv_ofs << raw_sensors.servo.l_hip_pitch.present_position << ",";
        csv_ofs << raw_sensors.servo.r_knee.present_position << ",";
        csv_ofs << raw_sensors.servo.l_knee.present_position << ",";
        csv_ofs << raw_sensors.servo.r_ankle_pitch.present_position << ",";
        csv_ofs << raw_sensors.servo.l_ankle_pitch.present_position << ",";
        csv_ofs << raw_sensors.servo.r_ankle_roll.present_position << ",";
        csv_ofs << raw_sensors.servo.l_ankle_roll.present_position << ",";
        csv_ofs << raw_sensors.servo.head_pan.present_position << ",";
        csv_ofs << raw_sensors.servo.head_tilt.present_position << "\n";

        // Write data to the file right away
        csv_ofs.flush();
    });

    on<Shutdown>().then([this] {
        csv_ofs.close();
        log<NUClear::WARN>("CSV ofstream closed!");
    });
}

}  // namespace module::tools
