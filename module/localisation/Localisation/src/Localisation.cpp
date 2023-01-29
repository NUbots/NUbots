#include "Localisation.hpp"

#include <Eigen/Dense>
#include <fstream>

#include "extension/Configuration.hpp"

namespace module::localisation {

    using extension::Configuration;

    using VisionGoal  = message::vision::Goal;
    using VisionGoals = message::vision::Goals;
    using message::support::FieldDescription;
    using VisionLines = message::vision::FieldLines;
    using message::localisation::Field;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;


    using utility::math::coordinates::cartesianToPolar;
    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::nusight::graph;


    Localisation::Localisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Localisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Localisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.grid_size   = config["grid_size"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            // Resize map to the field dimensions
            int map_width  = (fd.dimensions.field_width + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int map_length = (fd.dimensions.field_length + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
            fieldline_map.resize(map_width, map_length);

            // Calculate line width in grid cells
            int line_width = fd.dimensions.line_width / cfg.grid_size;

            // Add outer field lines
            int field_x0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int field_y0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int field_width  = (fd.dimensions.field_width) / cfg.grid_size;
            int field_length = (fd.dimensions.field_length) / cfg.grid_size;
            fieldline_map.add_rectangle(field_x0, field_y0, field_length, field_width, line_width);

            // Add left goal area box
            int left_goal_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int left_goal_box_y0 =
                (fd.dimensions.border_strip_min_width + (fd.dimensions.field_width - fd.dimensions.goal_area_width) / 2)
                / cfg.grid_size;
            int left_goal_box_width  = (fd.dimensions.goal_area_length) / cfg.grid_size;
            int left_goal_box_length = (fd.dimensions.goal_area_width) / cfg.grid_size;
            fieldline_map.add_rectangle(left_goal_box_x0,
                                        left_goal_box_y0,
                                        left_goal_box_width,
                                        left_goal_box_length,
                                        line_width);

            // Add right goal area box
            int right_goal_box_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.goal_area_length)
                / cfg.grid_size;
            int right_goal_box_y0     = left_goal_box_y0;
            int right_goal_box_width  = left_goal_box_width;
            int right_goal_box_length = left_goal_box_length;
            fieldline_map.add_rectangle(right_goal_box_x0,
                                        right_goal_box_y0,
                                        right_goal_box_width,
                                        right_goal_box_length,
                                        line_width);

            // Add left penalty area box
            int left_penalty_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int left_penalty_box_y0 = (fd.dimensions.border_strip_min_width
                                       + (fd.dimensions.field_width - fd.dimensions.penalty_area_width) / 2)
                                      / cfg.grid_size;
            int left_penalty_box_width  = (fd.dimensions.penalty_area_width) / cfg.grid_size;
            int left_penalty_box_length = (fd.dimensions.penalty_area_length) / cfg.grid_size;
            fieldline_map.add_rectangle(left_penalty_box_x0,
                                        left_penalty_box_y0,
                                        left_penalty_box_length,
                                        left_penalty_box_width,
                                        line_width);

            // Add right penalty area box
            int right_penalty_box_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.penalty_area_length)
                / cfg.grid_size;
            int right_penalty_box_y0     = left_penalty_box_y0;
            int right_penalty_box_width  = left_penalty_box_width;
            int right_penalty_box_length = left_penalty_box_length;
            fieldline_map.add_rectangle(right_penalty_box_x0,
                                        right_penalty_box_y0,
                                        right_penalty_box_length,
                                        right_penalty_box_width,
                                        line_width);

            // Add centre line
            int centre_line_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
            int centre_line_y0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
            int centre_line_width  = (fd.dimensions.field_width) / cfg.grid_size;
            int centre_line_length = (fd.dimensions.line_width) / cfg.grid_size;
            fieldline_map.add_rectangle(centre_line_x0,
                                        centre_line_y0,
                                        centre_line_length,
                                        centre_line_width,
                                        line_width);

            // TODO: Add centre cross in centre of field

            // TODO: Add left penalty cross in centre of penalty area

            // TODO: Add right penalty cross in centre of penalty area

            // Add centre circle
            int centre_circle_x0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
            int centre_circle_y0 =
                (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
            int centre_circle_r = (fd.dimensions.center_circle_diameter / 2) / cfg.grid_size;
            fieldline_map.add_circle(centre_circle_x0, centre_circle_y0, centre_circle_r, line_width);

            // --------------------- TEMPORARY: REMOVE LATER ---------------------
            // Open a file in write mode
            std::ofstream file("config/matrix.csv");
            // Write the matrix to the file
            file << fieldline_map.map;
            // Close the file
            file.close();

            // Initialise the particle filter
            state = Eigen::Vector3d::Zero();

            // Set the update times to now
            last_time_update_time        = NUClear::clock::now();
            last_measurement_update_time = NUClear::clock::now();
        });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& wc) {
            walk_engine_enabled = true;
            walk_command        = wc.command;
            log<NUClear::INFO>("Walk command received: ",
                               walk_command.x(),
                               ", ",
                               walk_command.y(),
                               ", ",
                               walk_command.z());
        });

        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
            walk_engine_enabled     = true;
            const auto current_time = NUClear::clock::now();
            last_time_update_time   = current_time;
        });

        on<Trigger<DisableWalkEngineCommand>>().then(
            [this](const DisableWalkEngineCommand& command) { walk_engine_enabled = false; });

        on<Trigger<StopCommand>>().then([this](const StopCommand& command) {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
            log<NUClear::INFO>("Stop command received");
        });

        on<Trigger<ExecuteGetup>>().then([this]() { falling = true; });

        on<Trigger<KillGetup>>().then([this]() { falling = false; });

        /* Perform time update */
        on<Every<TIME_UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then("Time Update", [this]() {
            if (walk_engine_enabled && !falling) {
                // Calculate the time since the last time update
                using namespace std::chrono;
                const auto current_time = NUClear::clock::now();
                const double dt         = duration_cast<duration<double>>(current_time - last_time_update_time).count();
                log<NUClear::DEBUG>("Time since last time update: ", dt);
                last_time_update_time = current_time;

                double delta_x     = walk_command.x() * dt;
                double delta_y     = walk_command.y() * dt;
                double delta_theta = walk_command.z() * dt;

                state.x() = state.x() + delta_x * cos(state.z() + delta_theta) - delta_y * sin(state.z() + delta_theta);
                state.y() = state.y() + delta_y * cos(state.z() + delta_theta) + delta_x * sin(state.z() + delta_theta);
                state.z() = state.z() + delta_theta;

                // Build and emit the field message
                auto field(std::make_unique<Field>());
                Eigen::Isometry2d position(Eigen::Isometry2d::Identity());
                position.translation() = Eigen::Vector2d(state.x(), state.y());
                position.linear()      = Eigen::Rotation2Dd(state.z()).toRotationMatrix();
                field->position        = position.matrix();

                log<NUClear::DEBUG>("State: ", state.x(), ", ", state.y(), ", ", state.z());
                emit(graph("State", state.x(), state.y(), state.z()));
                emit(field);
            }
        });

        on<Trigger<VisionLines>, With<FieldDescription>>().then(
            "Vision Lines",
            [this](const VisionLines& line_points, const FieldDescription& fd) {
                /* Perform measurement correction */

                // Identify the line points on the field
                for (auto point : line_points.points) {
                    Eigen::Isometry3d Hcw = Eigen::Isometry3d(line_points.Hcw.cast<double>());
                    auto rPCw             = ray2cartesian(point.cast<double>(), Hcw);
                    emit(graph("Field points:", rPCw.x(), rPCw.y(), rPCw.z()));
                    auto polar = cartesianToPolar(rPCw.head(2));
                    emit(graph("Field points polar:", polar.x(), polar.y()));
                }
                // TODO: Figure out how to incorporate these points into a measurement model.
                // Grid map?
                // Visual Mesh type grid map

                // Plug into particle filter

                // Profit.
            });
    }

    /// @brief Converts a ray from the camera to a point on the field
    /// @param uPCc ray from the camera to the field point
    /// @param Hcw the camera to world transform
    /// @return the field point in world coordinates
    Eigen::Vector3d Localisation::ray2cartesian(Eigen::Vector3d uPCc, Eigen::Isometry3d Hcw) {
        // Calculate the position of the field point relative to the robot
        auto Hwc  = Hcw.inverse();
        auto rWCw = -Hwc.translation();
        auto rPCw = uPCc * (rWCw.z() / uPCc.z());
        return rPCw;
    }

}  // namespace module::localisation
