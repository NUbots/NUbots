#include "HardwareIO.hpp"

#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/common/robot_shared.hpp>

#include "extension/Configuration.hpp"

namespace module::platform::Booster {

    using booster::robot::ChannelFactory;
    using booster::robot::b1::JointIndex;
    using extension::Configuration;
    using extension::behaviour::RunReason;
    using message::booster::BoosterFallDownState;
    using message::booster::BoosterGetUp;
    using message::booster::BoosterHeadRot;
    using message::booster::BoosterMode;
    using message::booster::BoosterVisualKick;
    using message::booster::BoosterWalk;
    using message::booster::FallDownStateType;
    using message::booster::K1Mode;
    using message::booster::VisualKickVer;
    using message::platform::RawSensors;

    static void fill_servo(RawSensors::Servo& servo, const booster_interface::msg::MotorState& motor) {
        servo.present_position = motor.q();
        servo.present_velocity = motor.dq();
        servo.present_current  = motor.tau_est();
        servo.temperature      = static_cast<float>(motor.temperature());
        servo.torque_enabled   = (motor.mode() != 0);
    }

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : NUClear::Reactor(std::move(environment)) {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this]() {
            log<INFO>("Starting Booster HardwareIO");
            ChannelFactory::Instance()->Init(0);

            booster_client.Init();
            booster_client.ChangeMode(RobotMode::kSoccer);

            low_state_channel = ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::LowState>(
                "rt/low_state",
                [this](const void* msg) { low_state_handler(msg); });

            battery_channel = ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::BatteryState>(
                "rt/battery_state",
                [this](const void* msg) { battery_handler(msg); });

            fall_down_channel = ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::FallDownState>(
                "rt/fall_down",
                [this](const void* msg) { fall_down_handler(msg); });

            button_event_channel =
                ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::ButtonEventMsg>(
                    "rt/button_event",
                    [this](const void* msg) { button_event_handler(msg); });
        });

        on<Shutdown>().then([this]() { booster_client.ChangeMode(RobotMode::kPrepare); });

        on<Trigger<BoosterWalk>>().then([this](const BoosterWalk& move) {
            if (move.velocity.isApprox(last_walk_velocity)) {
                return;
            }
            last_walk_velocity = move.velocity;
            log<DEBUG>("Sending move command with velocity: " + std::to_string(move.velocity.x()) + " "
                       + std::to_string(move.velocity.y()) + " " + std::to_string(move.velocity.z()));
            int32_t res = booster_client.Move(move.velocity.x(), move.velocity.y(), move.velocity.z());
            if (res != 0) {
                log<ERROR>("Failed to move: " + res_code_to_string(res));
            }
        });

        on<Trigger<BoosterHeadRot>>().then([this](const BoosterHeadRot& head) {
            if (head.rot.isApprox(last_head_rot)) {
                return;
            }
            last_head_rot = head.rot;
            log<DEBUG>("Sending head rotation command with yaw=" + std::to_string(head.rot.x())
                       + ", pitch=" + std::to_string(head.rot.y()));
            int32_t res = booster_client.RotateHead(head.rot.y(), head.rot.x());
            if (res != 0) {
                log<ERROR>("Failed to rotate head: " + res_code_to_string(res));
            }
        });

        on<Trigger<BoosterVisualKick>>().then([this](const BoosterVisualKick& kick) {
            booster::robot::b1::VisualKickVersion version;
            if (kick.version == VisualKickVer::V1) {
                version = booster::robot::b1::VisualKickVersion::kV1;
            }
            else if (kick.version == VisualKickVer::V2) {
                version = booster::robot::b1::VisualKickVersion::kV2;
            }
            else {
                version = booster::robot::b1::VisualKickVersion::kV1;
            }
            log<DEBUG>("Sending visual kick command: start=" + std::to_string(kick.start)
                       + ", version=" + std::to_string(int(version)));
            int32_t res = booster_client.VisualKick(kick.start, version);
            if (res != 0) {
                log<ERROR>("Failed to visual kick: " + res_code_to_string(res));
            }
        });

        on<Trigger<BoosterGetUp>>().then([this]() {
            int32_t res = booster_client.GetUpWithMode(RobotMode::kSoccer);
            if (res != 0) {
                log<ERROR>("Failed to get up: " + res_code_to_string(res));
            }
        });

        on<Trigger<BoosterMode>>().then([this](const BoosterMode& mode_msg) {
            RobotMode robot_mode;
            switch (static_cast<int>(mode_msg.mode)) {
                case K1Mode::DAMP: robot_mode = RobotMode::kDamping; break;
                case K1Mode::PREP: robot_mode = RobotMode::kPrepare; break;
                case K1Mode::WALK: robot_mode = RobotMode::kWalking; break;
                case K1Mode::CUSTOM: robot_mode = RobotMode::kCustom; break;
                case K1Mode::SOCCER: robot_mode = RobotMode::kSoccer; break;
                default: robot_mode = RobotMode::kSoccer; break;
            }
            int32_t res = booster_client.ChangeMode(robot_mode);
            if (res != 0) {
                log<ERROR>("Failed to change mode: " + res_code_to_string(res));
            }
        });
    }

    void HardwareIO::low_state_handler(const void* msg) {
        const auto* low_msg = static_cast<const booster_interface::msg::LowState*>(msg);

        auto sensors       = std::make_unique<RawSensors>();
        sensors->timestamp = NUClear::clock::now();

        // IMU
        const auto& imu            = low_msg->imu_state();
        sensors->accelerometer.x() = imu.acc()[0];
        sensors->accelerometer.y() = imu.acc()[1];
        sensors->accelerometer.z() = imu.acc()[2];
        sensors->gyroscope.x()     = imu.gyro()[0];
        sensors->gyroscope.y()     = imu.gyro()[1];
        sensors->gyroscope.z()     = imu.gyro()[2];

        // Serial chain motors: head (0-1), arms (2-9), waist (10)
        // Indices in the serial vector match JointIndex enum values directly.
        const auto& serial = low_msg->motor_state_serial();

        auto fill_serial = [&](RawSensors::Servo& servo, JointIndex idx) {
            auto i = static_cast<size_t>(idx);
            if (i < serial.size()) {
                fill_servo(servo, serial[i]);
            }
        };

        fill_serial(sensors->servo.head_pan, JointIndex::kHeadYaw);
        fill_serial(sensors->servo.head_tilt, JointIndex::kHeadPitch);
        fill_serial(sensors->servo.l_shoulder_pitch, JointIndex::kLeftShoulderPitch);
        fill_serial(sensors->servo.l_shoulder_roll, JointIndex::kLeftShoulderRoll);
        fill_serial(sensors->servo.l_elbow, JointIndex::kLeftElbowPitch);
        fill_serial(sensors->servo.r_shoulder_pitch, JointIndex::kRightShoulderPitch);
        fill_serial(sensors->servo.r_shoulder_roll, JointIndex::kRightShoulderRoll);
        fill_serial(sensors->servo.r_elbow, JointIndex::kRightElbowPitch);

        // Parallel mechanism motors: legs (indices 11-22 in JointIndex, stored as 0-11 in the vector)
        const auto& parallel    = low_msg->motor_state_parallel();
        const size_t leg_offset = static_cast<size_t>(JointIndex::kLeftHipPitch);

        auto fill_parallel = [&](RawSensors::Servo& servo, JointIndex idx) {
            auto i = static_cast<size_t>(idx) - leg_offset;
            if (i < parallel.size()) {
                fill_servo(servo, parallel[i]);
            }
        };

        fill_parallel(sensors->servo.l_hip_pitch, JointIndex::kLeftHipPitch);
        fill_parallel(sensors->servo.l_hip_roll, JointIndex::kLeftHipRoll);
        fill_parallel(sensors->servo.l_hip_yaw, JointIndex::kLeftHipYaw);
        fill_parallel(sensors->servo.l_knee, JointIndex::kLeftKneePitch);
        fill_parallel(sensors->servo.r_hip_pitch, JointIndex::kRightHipPitch);
        fill_parallel(sensors->servo.r_hip_roll, JointIndex::kRightHipRoll);
        fill_parallel(sensors->servo.r_hip_yaw, JointIndex::kRightHipYaw);
        fill_parallel(sensors->servo.r_knee, JointIndex::kRightKneePitch);

        // Battery SOC (updated by battery_handler)
        {
            std::lock_guard<std::mutex> lock(battery_mutex);
            sensors->battery = battery_soc;
        }

        // Buttons (updated by button_event_handler)
        {
            std::lock_guard<std::mutex> lock(buttons_mutex);
            sensors->buttons.left   = buttons.left;
            sensors->buttons.middle = buttons.middle;
            sensors->buttons.right  = buttons.right;
        }

        emit(std::move(sensors));
    }

    void HardwareIO::battery_handler(const void* msg) {
        const auto* bat_msg = static_cast<const booster_interface::msg::BatteryState*>(msg);
        std::lock_guard<std::mutex> lock(battery_mutex);
        battery_soc = bat_msg->soc();
    }

    void HardwareIO::fall_down_handler(const void* msg) {
        const auto* fd_msg = static_cast<const booster_interface::msg::FallDownState*>(msg);
        auto out           = std::make_unique<BoosterFallDownState>();
        switch (fd_msg->fall_down_state()) {
            case booster_interface::msg::IS_READY: out->fall_down_state = FallDownStateType::IS_READY; break;
            case booster_interface::msg::IS_FALLING: out->fall_down_state = FallDownStateType::IS_FALLING; break;
            case booster_interface::msg::HAS_FALLEN: out->fall_down_state = FallDownStateType::HAS_FALLEN; break;
            case booster_interface::msg::IS_GETTING_UP: out->fall_down_state = FallDownStateType::IS_GETTING_UP; break;
            default: out->fall_down_state = FallDownStateType::IS_READY; break;
        }
        out->is_recovery_available = fd_msg->is_recovery_available();
        emit(std::move(out));
    }

    void HardwareIO::button_event_handler(const void* msg) {
        const auto* btn_msg = static_cast<const booster_interface::msg::ButtonEventMsg*>(msg);
        const bool pressed  = (btn_msg->event() == booster_interface::msg::PRESS_DOWN
                               || btn_msg->event() == booster_interface::msg::LONG_PRESS_START
                               || btn_msg->event() == booster_interface::msg::LONG_PRESS_HOLD);

        std::lock_guard<std::mutex> lock(buttons_mutex);
        switch (btn_msg->button()) {
            case 0: buttons.left = pressed; break;
            case 1: buttons.middle = pressed; break;
            case 2: buttons.right = pressed; break;
            default: break;
        }
    }

}  // namespace module::platform::Booster
