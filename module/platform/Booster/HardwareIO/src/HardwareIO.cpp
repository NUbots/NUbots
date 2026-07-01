#include "HardwareIO.hpp"

#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/common/robot_shared.hpp>

#include "extension/Configuration.hpp"

#include "utility/math/comparison.hpp"

namespace module::platform::Booster {

    using booster::robot::ChannelFactory;
    using booster::robot::b1::GetModeResponse;
    using booster::robot::b1::JointIndexK1;
    using extension::Configuration;
    using extension::behaviour::RunReason;
    using message::booster::BoosterFallDownState;
    using message::booster::BoosterGetUp;
    using message::booster::BoosterHeadRot;
    using message::booster::BoosterMode;
    using message::booster::BoosterModeState;
    using message::booster::BoosterOdometry;
    using message::booster::BoosterVisualKick;
    using message::booster::BoosterWalk;
    using message::booster::FallDownStateType;
    using message::booster::K1Mode;
    using message::booster::VisualKickVer;
    using message::localisation::ResetFieldLocalisation;
    using message::platform::RawSensors;

    /// Internal event used to switch into prep mode once the gait has had time to stop.
    struct EnterPrep {};

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
            cfg.prep_settle_time =
                std::chrono::duration_cast<NUClear::clock::duration>(
                    std::chrono::duration<double>(config["prep_settle_time"].as<double>()));
        });

        on<Startup>().then([this]() {
            log<INFO>("Starting Booster HardwareIO");
            ChannelFactory::Instance()->Init(0);

            booster_client.Init();
            booster_client.ChangeMode(RobotMode::kPrepare);
            // Publish the initial mode so other modules can query it before the first poll
            publish_current_mode();

            if (int32_t res = booster_client.ResetOdometry(); res != 0) {
                log<ERROR>("Failed to reset odometry on startup: " + res_code_to_string(res));
            }

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

            odometer_channel = ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::Odometer>(
                "rt/odometer_state",
                [this](const void* msg) { odometer_handler(msg); });
        });

        on<Shutdown>().then([this]() { booster_client.ChangeMode(RobotMode::kPrepare); });

        on<Trigger<BoosterWalk>>().then([this](const BoosterWalk& move) {
            // While waiting to settle before switching to prep mode, ignore walk commands so the
            // robot isn't told to move again before it has stopped.
            if (prep_pending) {
                return;
            }
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
            // Clamp to the Booster SDK RotateHead limits (radians):
            //   pitch: downward positive, range [-0.3, 1.0]
            //   yaw:   leftward positive, range [-0.785, 0.785]
            constexpr double MIN_PITCH = -0.3;
            constexpr double MAX_PITCH = 1.0;
            constexpr double MIN_YAW   = -0.785;
            constexpr double MAX_YAW   = 0.785;

            const Eigen::Vector2d rot(utility::math::clamp(MIN_YAW, head.rot.x(), MAX_YAW),
                                      utility::math::clamp(MIN_PITCH, head.rot.y(), MAX_PITCH));

            if (rot.isApprox(last_head_rot)) {
                return;
            }
            last_head_rot = rot;
            log<DEBUG>("Sending head rotation command with yaw=" + std::to_string(rot.x())
                       + ", pitch=" + std::to_string(rot.y()));
            int32_t res = booster_client.RotateHead(rot.y(), rot.x());
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

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            int32_t res = booster_client.ResetOdometry();
            if (res != 0) {
                log<ERROR>("Failed to reset odometry: " + res_code_to_string(res));
            }
        });

        on<Trigger<BoosterMode>>().then([this](const BoosterMode& mode_msg) {
            // Switching into prep mid-stride makes the robot fall, so first command the gait to stop
            // and only switch to prep once it has had time to come to a complete stop.
            if (static_cast<int>(mode_msg.mode) == K1Mode::PREP) {
                log<DEBUG>("Stopping the gait before switching to prep mode");
                if (int32_t res = booster_client.Move(0.0, 0.0, 0.0); res != 0) {
                    log<ERROR>("Failed to stop before prep: " + res_code_to_string(res));
                }
                last_walk_velocity = Eigen::Vector3d::Zero();
                prep_pending       = true;
                emit<Scope::DELAY>(std::make_unique<EnterPrep>(), cfg.prep_settle_time);
                return;
            }

            // Any other mode request cancels a pending prep switch
            prep_pending = false;

            RobotMode robot_mode;
            switch (static_cast<int>(mode_msg.mode)) {
                case K1Mode::DAMP: robot_mode = RobotMode::kDamping; break;
                case K1Mode::WALK: robot_mode = RobotMode::kWalking; break;
                case K1Mode::CUSTOM: robot_mode = RobotMode::kCustom; break;
                case K1Mode::SOCCER: robot_mode = RobotMode::kSoccer; break;
                default: robot_mode = RobotMode::kSoccer; break;
            }
            change_mode(robot_mode);
        });

        on<Trigger<EnterPrep>>().then([this] {
            // The switch may have been cancelled by another mode request while the gait was stopping
            if (!prep_pending) {
                return;
            }
            prep_pending = false;
            change_mode(RobotMode::kPrepare);
        });

        // Periodically poll the robot for its actual motion mode so the published BoosterModeState
        // stays correct even if the mode changes outside of a BoosterMode command (or a change is
        // still in progress).
        on<Every<2, Per<std::chrono::seconds>>>().then([this] { publish_current_mode(); });
    }

    void HardwareIO::change_mode(RobotMode robot_mode) {
        int32_t res = booster_client.ChangeMode(robot_mode);
        if (res != 0) {
            log<ERROR>("Failed to change mode: " + res_code_to_string(res));
            return;
        }
        // Publish the mode the robot is now in so other modules can query the current mode
        publish_current_mode();
    }

    void HardwareIO::publish_current_mode() {
        GetModeResponse mode_response{};
        int32_t res = booster_client.GetMode(mode_response);
        if (res != 0) {
            log<WARN>("Failed to get current mode: " + res_code_to_string(res));
            return;
        }

        auto state = std::make_unique<BoosterModeState>();
        switch (mode_response.mode_) {
            case RobotMode::kDamping: state->mode = K1Mode::DAMP; break;
            case RobotMode::kPrepare: state->mode = K1Mode::PREP; break;
            case RobotMode::kWalking: state->mode = K1Mode::WALK; break;
            case RobotMode::kCustom: state->mode = K1Mode::CUSTOM; break;
            case RobotMode::kSoccer: state->mode = K1Mode::SOCCER; break;
            default: log<WARN>("Booster reported an unknown motion mode"); return;
        }
        emit(std::move(state));
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

        // Read joint-space feedback from the serial chain (the SDK converts the parallel ankle
        // mechanism for us). Indices match the JointIndex enum directly; the ankles occupy the
        // slots the enum labels kCrank{Up,Down}{Left,Right}.
        const auto& serial = low_msg->motor_state_serial();

        auto fill_serial = [&](RawSensors::Servo& servo, JointIndexK1 idx) {
            auto i = static_cast<size_t>(idx);
            if (i < serial.size()) {
                fill_servo(servo, serial[i]);
            }
        };

        fill_serial(sensors->servo.head_pan, JointIndexK1::kHeadYaw);
        fill_serial(sensors->servo.head_tilt, JointIndexK1::kHeadPitch);
        fill_serial(sensors->servo.l_shoulder_pitch, JointIndexK1::kLeftShoulderPitch);
        fill_serial(sensors->servo.l_shoulder_roll, JointIndexK1::kLeftShoulderRoll);
        fill_serial(sensors->servo.l_elbow, JointIndexK1::kLeftElbowPitch);
        fill_serial(sensors->servo.l_elbow_yaw, JointIndexK1::kLeftElbowYaw);
        fill_serial(sensors->servo.r_shoulder_pitch, JointIndexK1::kRightShoulderPitch);
        fill_serial(sensors->servo.r_shoulder_roll, JointIndexK1::kRightShoulderRoll);
        fill_serial(sensors->servo.r_elbow, JointIndexK1::kRightElbowPitch);
        fill_serial(sensors->servo.r_elbow_yaw, JointIndexK1::kRightElbowYaw);

        fill_serial(sensors->servo.l_hip_pitch, JointIndexK1::kLeftHipPitch);
        fill_serial(sensors->servo.l_hip_roll, JointIndexK1::kLeftHipRoll);
        fill_serial(sensors->servo.l_hip_yaw, JointIndexK1::kLeftHipYaw);
        fill_serial(sensors->servo.l_knee, JointIndexK1::kLeftKneePitch);
        fill_serial(sensors->servo.l_ankle_pitch, JointIndexK1::kCrankUpLeft);   // L ankle pitch
        fill_serial(sensors->servo.l_ankle_roll, JointIndexK1::kCrankDownLeft);  // L ankle roll
        fill_serial(sensors->servo.r_hip_pitch, JointIndexK1::kRightHipPitch);
        fill_serial(sensors->servo.r_hip_roll, JointIndexK1::kRightHipRoll);
        fill_serial(sensors->servo.r_hip_yaw, JointIndexK1::kRightHipYaw);
        fill_serial(sensors->servo.r_knee, JointIndexK1::kRightKneePitch);
        fill_serial(sensors->servo.r_ankle_pitch, JointIndexK1::kCrankUpRight);   // R ankle pitch
        fill_serial(sensors->servo.r_ankle_roll, JointIndexK1::kCrankDownRight);  // R ankle roll

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

    void HardwareIO::odometer_handler(const void* msg) {
        const auto* odo_msg = static_cast<const booster_interface::msg::Odometer*>(msg);
        auto out            = std::make_unique<BoosterOdometry>();
        out->x              = odo_msg->x();
        out->y              = odo_msg->y();
        out->theta          = odo_msg->theta();
        log<DEBUG>("Received odometry: x=" + std::to_string(out->x) + ", y=" + std::to_string(out->y)
                   + ", theta=" + std::to_string(out->theta));
        emit(out);
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
