/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "NMPCNavigation.hpp"


namespace module::planning {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Robots;
    using message::planning::VectorFieldVector;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    /**
     * @brief Converts an Eigen vector to an std::vector.
     * @param eigen_vec The Eigen vector to convert.
     * @param nlopt_vec The resulting std::vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void eigen_to_nlopt(const Eigen::Matrix<Scalar, n, 1>& eigen_vec, std::vector<Scalar>& nlopt_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        Eigen::Map<Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the Eigen vector to the std::vector view
        nlopt_eigen_vec = eigen_vec;
    }

    /**
     * @brief Converts an std::vector to an Eigen vector.
     * @param nlopt_vec The std::vector to convert.
     * @param eigen_vec The resulting Eigen vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void nlopt_to_eigen(const std::vector<Scalar>& nlopt_vec, Eigen::Matrix<Scalar, n, 1>& eigen_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        const Eigen::Map<const Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the std::vector view to the Eigen vector
        eigen_vec = nlopt_eigen_vec;
    }

    /**
     * @brief Type definition for an objective function that takes an Eigen vector as input and returns a scalar value.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     */
    template <typename Scalar, int nv>
    using ObjectiveFunction =
        std::function<Scalar(const Eigen::Matrix<Scalar, nv, 1>&, Eigen::Matrix<Scalar, nv, 1>&, void*)>;

    /**
     * @brief Wrapper function that converts input and output between NLopt and Eigen formats for an objective function.
     * @param n The size of the input vector.
     * @param x The input vector in NLopt format.
     * @param grad The gradient vector in NLopt format.
     * @param data Pointer to additional data that is passed to the objective function.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     * @return The scalar value of the objective function.
     */
    template <typename Scalar, int nv>
    inline Scalar eigen_objective_wrapper(unsigned n, const Scalar* x, Scalar* grad, void* data) {
        ObjectiveFunction<Scalar, nv>& obj_fun = *static_cast<ObjectiveFunction<Scalar, nv>*>(data);

        // Convert input from NLopt format to Eigen format
        Eigen::Map<const Eigen::Matrix<Scalar, nv, 1>> eigen_x(x, n);
        Eigen::Matrix<Scalar, nv, 1> eigen_grad = Eigen::Matrix<Scalar, nv, 1>::Zero();

        if (grad) {
            eigen_grad.resize(n);
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        // Call the actual objective function implemented with Eigen
        Scalar result = obj_fun(eigen_x, eigen_grad, data);

        if (grad) {
            // Copy the gradient back to NLopt format
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        return result;
    }

    NMPCNavigation::NMPCNavigation(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("NMPCNavigation.yaml").then([this](const Configuration& config) {
            // Load configuration parameters
            cfg.dt = config["dt"].as<double>();
            cfg.Q  = Eigen::Vector3d(config["Q"].as<Expression>()).asDiagonal();
            cfg.R  = Eigen::Vector3d(config["R"].as<Expression>()).asDiagonal();
            log<NUClear::INFO>("NMPCNavigation configuration loaded",
                               "dt: ",
                               cfg.dt,
                               "\n Q: \n",
                               cfg.Q,
                               "\n R: \n",
                               cfg.R);
            cfg.max_velocity_x       = config["max_velocity_x"].as<double>();
            cfg.max_velocity_y       = config["max_velocity_y"].as<double>();
            cfg.max_angular_velocity = config["max_angular_velocity"].as<double>();
            cfg.obstacle_weight      = config["obstacle_weight"].as<double>();
            cfg.robot_radius         = config["robot_radius"].as<double>();
            cfg.obstacle_radius      = config["obstacle_radius"].as<double>();
        });

        on<Provide<WalkTo>, With<Sensors>, Optional<With<Robots>>>().then(
            [this](const WalkTo& walk_to, const Sensors& sensors, const std::shared_ptr<const Robots>& robots) {
                // Get current state
                Eigen::Vector3d initial_state = Eigen::Vector3d::Zero();  // get_current_state(sensors);

                // Get target state
                Eigen::Vector3d target_state = get_target_state(walk_to);

                // Get obstacles
                std::vector<Eigen::Vector2d> obstacles = get_obstacles(robots, sensors);

                // Solve NMPC problem
                Eigen::VectorXd optimal_actions = solve_nmpc(initial_state, target_state, obstacles);

                // Extract first action and apply it
                Eigen::Vector3d velocity_command = optimal_actions.segment<3>(0);
                emit<Task>(std::make_unique<Walk>(velocity_command));

                // Emit debug information
                emit_debug_info(initial_state, target_state, optimal_actions);
            });
    }

    // Eigen::Vector3d NMPCNavigation::get_current_state(const Sensors& sensors) {
    //     // Extract current position and orientation from sensors
    //     Eigen::Vector3d current_state;
    //     Eigen::Isometry3d Hwr   = sensors.Hrw.inverse();
    //     current_state.head<2>() = Hwr.translation().head<2>();
    //     current_state(2)        = std::atan2(Hwr.linear.col(0).y(), Hwr.linear.col(0).x());
    //     return current_state;
    // }

    Eigen::Vector3d NMPCNavigation::get_target_state(const WalkTo& walk_to) {
        // Extract target position and orientation from WalkTo message
        Eigen::Vector3d target_state;
        target_state.head<2>() = walk_to.Hrd.translation().head<2>();
        target_state(2)        = std::atan2(walk_to.Hrd.linear().col(0).y(), walk_to.Hrd.linear().col(0).x());
        return target_state;
    }

    std::vector<Eigen::Vector2d> NMPCNavigation::get_obstacles(const std::shared_ptr<const Robots>& robots,
                                                               const Sensors& sensors) {
        std::vector<Eigen::Vector2d> obstacles;
        if (robots) {
            for (const auto& robot : robots->robots) {
                obstacles.push_back((sensors.Hrw * robot.rRWw).head<2>());
            }
        }
        return obstacles;
    }

    Eigen::VectorXd NMPCNavigation::solve_nmpc(const Eigen::Vector3d& initial_state,
                                               const Eigen::Vector3d& target_state,
                                               const std::vector<Eigen::Vector2d>& obstacles) {
        ObjectiveFunction<double, n_opt_vars> obj_fun = [&](const Eigen::Matrix<double, n_opt_vars, 1>& x,
                                                            Eigen::Matrix<double, n_opt_vars, 1>& grad,
                                                            void* data) -> double {
            (void) data;  // Unused in this case
            (void) grad;  // Unused in this case

            double cost = cost_function(x, grad, initial_state, target_state, obstacles);

            return cost;
        };

        // Set up NLopt optimizer
        nlopt::algorithm algorithm = nlopt::LN_COBYLA;
        nlopt::opt opt(algorithm, n_opt_vars);

        // Set objective function
        opt.set_min_objective(eigen_objective_wrapper<double, n_opt_vars>, &obj_fun);

        // // Set bounds
        std::vector<double> lb(n_opt_vars, -std::numeric_limits<double>::infinity());
        std::vector<double> ub(n_opt_vars, std::numeric_limits<double>::infinity());
        for (int i = 0; i < horizon; ++i) {
            lb[i * 3]     = -cfg.max_velocity_x;
            lb[i * 3 + 1] = -cfg.max_velocity_y;
            lb[i * 3 + 2] = -cfg.max_angular_velocity;
            ub[i * 3]     = cfg.max_velocity_x;
            ub[i * 3 + 1] = cfg.max_velocity_y;
            ub[i * 3 + 2] = cfg.max_angular_velocity;
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // Set stopping criteria
        opt.set_xtol_rel(1e-4);
        opt.set_ftol_rel(1e-4);
        opt.set_maxeval(100);

        // Optimize
        double min_cost;
        std::vector<double> optimal_action_sequence = std::vector<double>(n_opt_vars, 0.0);
        nlopt::result result                        = opt.optimize(optimal_action_sequence, min_cost);

        // Convert result to Eigen vector
        Eigen::VectorXd optimal_actions =
            Eigen::Map<Eigen::VectorXd>(optimal_action_sequence.data(), optimal_action_sequence.size());

        return optimal_actions;
    }

    double NMPCNavigation::cost_function(const Eigen::Matrix<double, n_opt_vars, 1>& x,
                                         Eigen::Matrix<double, n_opt_vars, 1>& grad,
                                         const Eigen::Vector3d& initial_state,
                                         const Eigen::Vector3d& target_state,
                                         const std::vector<Eigen::Vector2d>& obstacles) {
        (void) grad;  // Unused for now, we're not providing gradients

        double cost           = 0.0;
        Eigen::Vector3d state = initial_state;

        for (int i = 0; i < horizon; ++i) {
            Eigen::Vector3d action = x.segment<3>(i * 3);

            // State cost
            Eigen::Vector3d state_error = state - target_state;
            cost += state_error.transpose() * cfg.Q * state_error;

            // Control cost
            cost += action.transpose() * cfg.R * action;

            // Obstacle cost
            for (const auto& obstacle : obstacles) {
                double distance = (state.head<2>() - obstacle).norm() - cfg.robot_radius - cfg.obstacle_radius;
                if (distance < 0) {
                    cost += cfg.obstacle_weight * std::pow(distance, 2);
                }
            }

            // Update state (simple kinematic model)
            double dx = action(0) * std::cos(state(2)) - action(1) * std::sin(state(2));
            double dy = action(0) * std::sin(state(2)) + action(1) * std::cos(state(2));
            state(0)  = state(0) + dx * cfg.dt;
            state(1)  = state(1) + dy * cfg.dt;
            state(2)  = state(2) + action(2) * cfg.dt;
        }

        emit(graph("NMPC Final State", state(0), state(1), state(2)));
        log<NUClear::INFO>("NMPC cost: ", cost);
        return cost;
    }

    void NMPCNavigation::emit_debug_info(const Eigen::Vector3d& initial_state,
                                         const Eigen::Vector3d& target_state,
                                         const Eigen::VectorXd& optimal_actions) {
        auto debug               = std::make_unique<WalkToDebug>();
        debug->Hrd               = Eigen::Isometry3d::Identity();
        debug->Hrd.translation() = target_state.head<3>();
        debug->Hrd.linear()      = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, target_state(2)));

        debug->velocity_target = optimal_actions.segment<3>(0);

        // // Predicted trajectory
        // Eigen::Vector3d state = initial_state;
        // for (int i = 0; i < horizon; ++i) {
        //     Eigen::Vector3d action = optimal_actions.segment<3>(i * 3);
        //     state.head<2>() += action.head<2>() * cfg.dt;
        //     state(2) += action(2) * cfg.dt;

        //     VectorFieldVector vector;
        //     vector.rVRr      = state;
        //     vector.direction = std::atan2(action(1), action(0));
        //     vector.magnitude = action.head<2>().norm();
        //     debug->vector_field.push_back(vector);
        // }

        emit(debug);

        // Additional debugging graphs
        emit(graph("NMPC Initial State", initial_state.x(), initial_state.y(), initial_state.z()));
        emit(graph("NMPC Target State", target_state.x(), target_state.y(), target_state.z()));
        emit(graph("NMPC Optimal Action", optimal_actions(0), optimal_actions(1), optimal_actions(2)));
    }

}  // namespace module::planning
