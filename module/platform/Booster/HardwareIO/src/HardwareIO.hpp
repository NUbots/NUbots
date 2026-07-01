#ifndef MODULE_PLATFORM_BOOSTER_HARDWAREIO_HPP
#define MODULE_PLATFORM_BOOSTER_HARDWAREIO_HPP

#include <booster/idl/b1/BatteryState.h>
#include <booster/idl/b1/ButtonEvent.h>
#include <booster/idl/b1/FallDownState.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/Odometer.h>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <mutex>
#include <nuclear>
#include <string>

#include "extension/Behaviour.hpp"

#include "message/booster/BoosterFallDownState.hpp"
#include "message/booster/BoosterGetUp.hpp"
#include "message/booster/BoosterHeadRot.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterModeState.hpp"
#include "message/booster/BoosterOdometry.hpp"
#include "message/booster/BoosterVisualKick.hpp"
#include "message/booster/BoosterWalk.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"


namespace module::platform::Booster {

    class HardwareIO : public NUClear::Reactor {
    private:
        struct Config {
            /// @brief Time to wait after commanding the gait to stop before switching into prep mode,
            /// so the robot comes to a complete stop and does not fall from switching mid-stride
            NUClear::clock::duration prep_settle_time{};
        } cfg;

        /// @brief Whether a switch into prep mode is pending (waiting for the gait to stop). Used to
        /// cancel the deferred switch if another mode is requested in the meantime.
        bool prep_pending = false;

        struct Buttons {
            bool left   = false;
            bool middle = false;
            bool right  = false;
        } buttons;
        std::mutex buttons_mutex;

        float battery_soc = 0.0f;
        std::mutex battery_mutex;

        Eigen::Vector3d last_walk_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector2d last_head_rot      = Eigen::Vector2d::Zero();

        booster::robot::ChannelPtr<booster_interface::msg::LowState> low_state_channel;
        booster::robot::ChannelPtr<booster_interface::msg::BatteryState> battery_channel;
        booster::robot::ChannelPtr<booster_interface::msg::FallDownState> fall_down_channel;
        booster::robot::ChannelPtr<booster_interface::msg::ButtonEventMsg> button_event_channel;
        booster::robot::ChannelPtr<booster_interface::msg::Odometer> odometer_channel;

        booster::robot::b1::B1LocoClient booster_client;

        void low_state_handler(const void* msg);
        void fall_down_handler(const void* msg);
        void battery_handler(const void* msg);
        void button_event_handler(const void* msg);
        void odometer_handler(const void* msg);

        /// Query the robot's current motion mode from the SDK and publish it as a BoosterModeState so
        /// other modules can read the actual mode the robot is in.
        void publish_current_mode();

        /// Change the robot's motion mode and publish the resulting mode.
        void change_mode(booster::robot::RobotMode robot_mode);

        static std::string res_code_to_string(int32_t res_code) {
            std::string out;
            switch (res_code) {
                case 0: out = "Success"; break;
                case 100: out = "Request timed out"; break;
                case 400: out = "Bad request"; break;
                case 409: out = "Request conflict"; break;
                case 500: out = "Internal server error"; break;
                case 501: out = "Request rejected"; break;
                default: out = "UNKNOWN"; break;
            }
            return out + " (" + std::to_string(res_code) + ")";
        }

    public:
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::platform::Booster

#endif  // MODULE_PLATFORM_BOOSTER_HARDWAREIO_HPP
