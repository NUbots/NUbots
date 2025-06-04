#ifndef MODULE_TOOLS_WALKINGREFERENCEGENERATOR_HPP
#define MODULE_TOOLS_WALKINGREFERENCEGENERATOR_HPP

#include <json.hpp>
#include <nuclear>
#include <tinyrobotics/inversekinematics.hpp>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/math.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/actuation/ServoCommand.hpp"

#include "utility/actuation/tinyrobotics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/skill/WalkGenerator.hpp"

namespace module::tools {

    using utility::input::LimbID;
    using utility::input::ServoID;

    using tinyrobotics::InverseKinematicsMethod;

    inline std::vector<std::pair<int, ServoID>> joint_map = {
        {0, ServoID::L_ANKLE_ROLL},     {1, ServoID::L_ANKLE_PITCH},     {2, ServoID::L_KNEE},
        {3, ServoID::L_HIP_PITCH},      {4, ServoID::L_HIP_ROLL},        {5, ServoID::L_HIP_YAW},
        {6, ServoID::R_ANKLE_ROLL},     {7, ServoID::R_ANKLE_PITCH},     {8, ServoID::R_KNEE},
        {9, ServoID::R_HIP_PITCH},      {10, ServoID::R_HIP_ROLL},       {11, ServoID::R_HIP_YAW},
        {12, ServoID::HEAD_PITCH},      {13, ServoID::HEAD_YAW},         {14, ServoID::L_ELBOW},
        {15, ServoID::L_SHOULDER_ROLL}, {16, ServoID::L_SHOULDER_PITCH}, {17, ServoID::R_ELBOW},
        {18, ServoID::R_SHOULDER_ROLL}, {19, ServoID::R_SHOULDER_PITCH}};

    // servo id to joint id map
    inline std::map<ServoID, int> servo_id_to_joint_id = {
        {ServoID::L_ANKLE_ROLL, 0},     {ServoID::L_ANKLE_PITCH, 1},     {ServoID::L_KNEE, 2},
        {ServoID::L_HIP_PITCH, 3},      {ServoID::L_HIP_ROLL, 4},        {ServoID::L_HIP_YAW, 5},
        {ServoID::R_ANKLE_ROLL, 6},     {ServoID::R_ANKLE_PITCH, 7},     {ServoID::R_KNEE, 8},
        {ServoID::R_HIP_PITCH, 9},      {ServoID::R_HIP_ROLL, 10},       {ServoID::R_HIP_YAW, 11},
        {ServoID::HEAD_PITCH, 12},      {ServoID::HEAD_YAW, 13},         {ServoID::L_ELBOW, 14},
        {ServoID::L_SHOULDER_ROLL, 15}, {ServoID::L_SHOULDER_PITCH, 16}, {ServoID::R_ELBOW, 17},
        {ServoID::R_SHOULDER_ROLL, 18}, {ServoID::R_SHOULDER_PITCH, 19}};


    class WalkingReferenceGenerator : public NUClear::Reactor {

    private:
        /// @brief Stores configuration values
        struct Config {
            std::string file_path;

            /// @brief Stores the gains for each servo
            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states{};

            /// @brief Desired arm positions while walking
            std::vector<std::pair<utility::input::ServoID, double>> arm_positions{};

            /// @brief Walk engine parameters
            utility::skill::WalkGenerator<double>::WalkParameters walk_generator_parameters{};

            /// @brief Ranges for the walk reference generator
            struct Ranges {
                double increment;
                std::vector<double> x;
                std::vector<double> y;
                std::vector<double> yaw;
            } ranges;

            /// @brief Time step for the walk reference generator
            double sampling_time_step;

            /// @brief Path to the URDF file
            std::string urdf_path = "";

            /// @brief Name of torso link in tinyrobotics model
            std::string torso_name = "torso";

            /// @brief Name of left foot base link in tinyrobotics model
            std::string left_foot_name = "left_foot_base";

            /// @brief Name of right foot base link in tinyrobotics model
            std::string right_foot_name = "right_foot_base";
        } cfg;

        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 100;

        /// @brief Last time we updated the walk engine
        NUClear::clock::time_point last_update_time{};

        /// @brief Generates swing foot and torso trajectories for given walk velocity target
        utility::skill::WalkGenerator<double> walk_generator{};

        /// @brief Kinematics model
        message::actuation::KinematicsModel kinematics_model{};

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_joints = 20;

        /// @brief tinyrobotics model of the NUgus robot for left leg IK
        tinyrobotics::Model<double, n_joints> nugus_model_left;

        /// @brief tinyrobotics model of the NUgus robot for right leg IK
        tinyrobotics::Model<double, n_joints> nugus_model_right;

        /// @brief tinyrobotics inverse kinematics options
        tinyrobotics::InverseKinematicsOptions<double, n_joints> options;

        /// @brief Generate trajectory data for a given velocity target
        nlohmann::json generate_trajectory(const Eigen::Vector3d& velocity_target);

        /**
         * @brief Converts a string to an InverseKinematicsMethod
         *
         * @param method_string
         * @return InverseKinematicsMethod
         */
        InverseKinematicsMethod ik_string_to_method(const std::string& method_string);

    public:
        /// @brief Called by the powerplant to build and setup the WalkingReferenceGenerator reactor.
        explicit WalkingReferenceGenerator(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_WALKINGREFERENCEGENERATOR_HPP
