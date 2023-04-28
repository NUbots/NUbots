#ifndef MODULE_PURPOSE_KEYBOARDWALK_HPP
#define MODULE_PURPOSE_KEYBOARDWALK_HPP

#include <Eigen/Core>
#include <mutex>

// clang-format off
// This include needs to come immediately before the OK undef
#include <ncurses.h>
// because ncurses defines OK. We don't need (or want) it.
#undef OK
// clang-format on

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "utility/input/LimbID.hpp"

namespace module::purpose {

    enum class LogColours : short {
        TRACE_COLOURS = 1,
        DEBUG_COLOURS = 2,
        INFO_COLOURS  = 3,
        WARN_COLOURS  = 4,
        ERROR_COLOURS = 5,
        FATAL_COLOURS = 6
    };

    class KeyboardWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        /// @brief Increments the value of walk command (dx, dy) by this amount when a key is pressed
        static constexpr const float DIFF = 0.01f;

        /// @brief Increments the value of walk command (dtheta) by this amount when a key is pressed
        static constexpr const float ROT_DIFF = 0.1f;

        /// @brief Increments the value of head yaw/pitch by this amount when a key is pressed
        static constexpr const float HEAD_DIFF = 1.0f * float(M_PI) / 180.0f;


        bool walk_enabled = false;

        /// @brief Walk command (dx, dy, dtheta)
        Eigen::Vector3f walk_command = Eigen::Vector3f::Zero();

        /// @brief Desired head yaw
        float head_yaw = 0.0f;

        /// @brief Desired head pitch
        float head_pitch = 0.0f;


        std::shared_ptr<WINDOW> command_window;
        std::shared_ptr<WINDOW> log_window;
        bool colours_enabled;

        std::mutex mutex;

        void create_windows();
        void forward();
        void left();
        void back();
        void right();
        void turn_left();
        void turn_right();
        void get_up();
        void reset();
        void kick(utility::input::LimbID::Value l);
        void look_left();
        void look_right();
        void look_up();
        void look_down();
        void walk_toggle();

        void update_command();
        void print_status();
        void update_window(const std::shared_ptr<WINDOW>& window,
                           const LogColours& colours,
                           const std::string& source,
                           const std::string& message,
                           const bool& print_level);

    public:
        /// @brief Called by the powerplant to build and setup the KeyboardWalk reactor.
        explicit KeyboardWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SOCCER_HPP
