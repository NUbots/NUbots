#include "SensorFilter.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/actuation/tinyrobotics.hpp"
#include "utility/input/FrameID.hpp"

namespace module::input {

    using message::actuation::BodySide;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::RawSensors;

    using utility::actuation::tinyrobotics::forward_kinematics_to_servo_map;
    using utility::actuation::tinyrobotics::sensors_to_configuration;
    using utility::input::FrameID;

    void SensorFilter::update_kinematics(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // Convert the sensor joint angles to a configuration vector
        Eigen::Matrix<double, n_servos, 1> q = sensors_to_configuration<double, n_servos>(sensors);

        // **************** Kinematics ****************
        // Htx is a map from FrameID to homogeneous transforms from each frame to the torso
        std::vector<Eigen::Isometry3d> fk = tinyrobotics::forward_kinematics(nugus_model, q);
        auto Htx                          = forward_kinematics_to_servo_map(fk);
        for (const auto& entry : Htx) {
            sensors->Htx[entry.first] = entry.second.matrix();
        }

        // **************** Centre of Mass  ****************
        sensors->rMTt = tinyrobotics::center_of_mass(nugus_model, q);

        // **************** Foot Down Information ****************
        if (cfg.foot_down.method == "Z_HEIGHT") {
            const Eigen::Isometry3d Htr(sensors->Htx[FrameID::R_FOOT_BASE]);
            const Eigen::Isometry3d Htl(sensors->Htx[FrameID::L_FOOT_BASE]);
            const Eigen::Isometry3d Hlr = Htl.inverse() * Htr;
            const double rRLl_z         = Hlr.translation().z();

            // The right foot is not down if the z height if the right foot is above a threshold (in left foot space)
            sensors->feet[BodySide::RIGHT].down = rRLl_z > cfg.foot_down.threshold ? false : true;
            // The left foot is not down if the z height if the right foot is below a threshold (in left foot space)
            sensors->feet[BodySide::LEFT].down = rRLl_z < -cfg.foot_down.threshold ? false : true;
        }
        else if (cfg.foot_down.method == "FSR") {
            // Determine if any two diagonally opposite FSRs are in contact with the ground, which is either fsr1
            // and fsr3, or fsr2 and fsr4. A FSR is in contact with the ground if its value is greater than the
            // certainty threshold
            auto is_foot_down = [](const auto& fsr, const double threshold) {
                return (fsr.fsr1 > threshold && fsr.fsr3 > threshold) || (fsr.fsr2 > threshold && fsr.fsr4 > threshold);
            };
            sensors->feet[BodySide::LEFT].down  = is_foot_down(raw_sensors.fsr.left, cfg.foot_down.threshold);
            sensors->feet[BodySide::RIGHT].down = is_foot_down(raw_sensors.fsr.right, cfg.foot_down.threshold);
        }
        else {
            log<WARN>("Unknown foot down method");
        }

        // **************** Planted Foot Information ****************
        bool both = sensors->feet[BodySide::LEFT].down && sensors->feet[BodySide::RIGHT].down;
        bool left = sensors->feet[BodySide::LEFT].down && !sensors->feet[BodySide::RIGHT].down;
        sensors->planted_foot_phase =
            both ? WalkState::Phase::DOUBLE : (left ? WalkState::Phase::LEFT : WalkState::Phase::RIGHT);
    }
}  // namespace module::input
