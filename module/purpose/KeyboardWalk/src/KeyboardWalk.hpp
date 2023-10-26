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

    /// @brief Enum for log colours
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
        /// @brief Increments the value of walk command (dx, dy) by this amount when a key is pressed
        static constexpr const double DIFF = 0.01f;

        /// @brief Increments the value of walk command (dtheta) by this amount when a key is pressed
        static constexpr const double ROT_DIFF = 0.1f;

        /// @brief Increments the value of head yaw/pitch by this amount when a key is pressed
        static constexpr const double HEAD_DIFF = 1.0f * double(M_PI) / 180.0f;

        /// @brief Whether or not the walk is enabled
        bool walk_enabled = false;

        /// @brief Walk command (dx, dy, dtheta)
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Desired head yaw
        double head_yaw = 0.0f;

        /// @brief Desired head pitch
        double head_pitch = 0.0f;

        /// @brief Command window
        std::shared_ptr<WINDOW> command_window;

        /// @brief Log window
        std::shared_ptr<WINDOW> log_window;

        /// @brief Whether or not colours are enabled
        bool colours_enabled;

        /// @brief Mutex to ensure that only one thread is writing to the log window at a time
        std::mutex mutex;

        /// @brief Creates windows for the command and log
        void create_windows();

        /// @brief Increments the value of walk command (dx) by DIFF
        void forward();

        /// @brief Increments the value of walk command (dy) by DIFF
        void left();

        /// @brief Increments the value of walk command (dx) by -DIFF
        void back();

        /// @brief Increments the value of walk command (dy) by -DIFF
        void right();

        /// @brief  Increments the value of walk command (dtheta) by ROT_DIFF
        void turn_left();

        /// @brief  Increments the value of walk command (dtheta) by -ROT_DIFF
        void turn_right();

        /// @brief Resets the walk command to zero
        void reset();

        /// @brief Quits the program
        void quit();

        /// @brief Executes Clap Open move
        void dab();

        /// @brief Executes Clap Close move
        void clap();

        /// @brief Executes Overhead Thrust Right move
        void arm_dangle();

        /// @brief Executes Overhead Thrust Left move
        void crouch();

        /// @brief Executes Star 1 move
        void raise_the_roof();

        /// @brief Executes Star 2 move
        void star();

        /// @brief Executes Crouch 1 move
        void the_robot();

        /// @brief Executes Crouch 2 move
        void wave();

        /// @brief Emits task to kick with specified leg
        /// @param l Leg to kick with (LEFT or RIGHT)
        void kick(utility::input::LimbID::Value l);

        /// @brief Increments the value of head yaw by HEAD_DIFF
        void look_left();

        /// @brief Increments the value of head yaw by -HEAD_DIFF
        void look_right();

        /// @brief Increments the value of head pitch by HEAD_DIFF
        void look_up();

        /// @brief Increments the value of head pitch by -HEAD_DIFF
        void look_down();

        /// @brief Toggles the walk on/off
        void walk_toggle();

        /// @brief Emits task to walk with current walk command
        void update_command();

        /// @brief Prints the current status of the walk command
        void print_status();

        /// @brief Updates the window with the specified message
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

#endif  // MODULE_PURPOSE_KEYBOARDWALK_HPP
