/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
#ifndef UTILITY_STRATEGY_FORMATION_HPP
#define UTILITY_STRATEGY_FORMATION_HPP

#include <Eigen/Core>
#include <algorithm>
#include <json.hpp>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>

#include "message/support/FieldDescription.hpp"

namespace utility::strategy::formation {

    using Dims = message::support::FieldDescription::FieldDimensions;

    // Named field dimensions used by {"field": "fieldLength", ...} expressions.
    inline std::unordered_map<std::string, double> field_dimensions(const Dims& d) {
        return {
            {"fieldLength", d.field_length},
            {"fieldWidth", d.field_width},
            {"penaltyAreaLength", d.penalty_area_length},
            {"penaltyAreaWidth", d.penalty_area_width},
            {"goalAreaLength", d.goal_area_length},
            {"goalAreaWidth", d.goal_area_width},
            {"penaltyMarkDistance", d.penalty_mark_distance},
            {"centerCircleDiameter", d.center_circle_diameter},
        };
    }

    // Named field positions used by {"position": "field_min_x", ...} expressions.
    inline std::unordered_map<std::string, double> field_positions(const Dims& d) {
        return {
            {"field_min_x", -d.field_length / 2.0},
            {"field_max_x", d.field_length / 2.0},
            {"field_min_y", -d.field_width / 2.0},
            {"field_max_y", d.field_width / 2.0},
            {"left_penalty_area_max_x", -d.field_length / 2.0 + d.penalty_area_length},
            {"right_penalty_area_min_x", d.field_length / 2.0 - d.penalty_area_length},
            {"penalty_area_min_y", -d.penalty_area_width / 2.0},
            {"penalty_area_max_y", d.penalty_area_width / 2.0},
            {"left_goal_area_max_x", -d.field_length / 2.0 + d.goal_area_length},
            {"right_goal_area_min_x", d.field_length / 2.0 - d.goal_area_length},
            {"goal_area_min_y", -d.goal_area_width / 2.0},
            {"goal_area_max_y", d.goal_area_width / 2.0},
            {"left_penalty_mark_x", -d.field_length / 2.0 + d.penalty_mark_distance},
            {"right_penalty_mark_x", d.field_length / 2.0 - d.penalty_mark_distance},
        };
    }

    // Evaluate a JSON measure expression (number, field-dimension, position, or op) to a double.
    inline std::optional<double> resolve_measure(const nlohmann::json& value, const Dims& d) {
        if (value.is_number()) {
            return value.get<double>();
        }
        if (!value.is_object()) {
            return std::nullopt;
        }
        if (value.contains("field")) {
            auto dims = field_dimensions(d);
            auto it   = dims.find(value["field"].get<std::string>());
            if (it == dims.end()) {
                return std::nullopt;
            }
            return it->second * value.value("scale", 1.0) + value.value("offset", 0.0);
        }
        if (value.contains("position")) {
            auto pos = field_positions(d);
            auto it  = pos.find(value["position"].get<std::string>());
            if (it == pos.end()) {
                return std::nullopt;
            }
            return it->second + value.value("offset", 0.0);
        }
        if (value.contains("op") && value.contains("terms")) {
            std::string op = value["op"].get<std::string>();
            std::vector<double> terms{};
            for (const auto& t : value["terms"]) {
                auto v = resolve_measure(t, d);
                if (!v.has_value()) {
                    return std::nullopt;
                }
                terms.push_back(*v);
            }
            if (op == "negate" && terms.size() == 1) {
                return -terms[0];
            }
            if (terms.size() >= 2) {
                if (op == "add") {
                    double r = 0.0;
                    for (double t : terms) {
                        r += t;
                    }
                    return r;
                }
                if (op == "subtract") {
                    return terms[0] - terms[1];
                }
                if (op == "multiply") {
                    double r = 1.0;
                    for (double t : terms) {
                        r *= t;
                    }
                    return r;
                }
            }
        }
        return std::nullopt;
    }

    // Whether the kicking team is "us", "them", or "none".
    inline std::string resolve_kicking_relation(std::optional<int> kicking_team, int own_team_id) {
        if (!kicking_team.has_value()) {
            return "none";
        }
        return (*kicking_team == own_team_id) ? "us" : "them";
    }

    // Map game state strings to the legacy mode key used in the formation JSON.
    inline std::string resolve_legacy_mode(const std::string& game_phase,
                                           const std::string& state,
                                           const std::string& set_play,
                                           const std::string& relation) {
        if (game_phase == "timeout") {
            return "timeout";
        }
        if (game_phase == "penalty_shoot_out") {
            return (relation == "us") ? "penalty_kick_us" : "penalty_kick_them";
        }
        if (set_play != "none" && relation != "none") {
            return set_play + "_" + relation;
        }
        if (set_play == "none" && state != "playing" && relation != "none") {
            return "kickoff_" + relation;
        }
        return "normal_play";
    }

    // Find the active mode object. Tries the full advertised key, then the legacy key, then "normal_play".
    inline const nlohmann::json* resolve_mode(const nlohmann::json& formation,
                                              const std::string& game_phase,
                                              const std::string& state,
                                              const std::string& set_play,
                                              std::optional<int> kicking_team,
                                              int own_team_id) {
        if (!formation.contains("modes")) {
            return nullptr;
        }
        const auto& modes    = formation["modes"];
        std::string relation = resolve_kicking_relation(kicking_team, own_team_id);
        std::string legacy   = resolve_legacy_mode(game_phase, state, set_play, relation);

        std::string advertised = "advertised__phase_" + game_phase + "__state_" + state
                                 + "__set_play_" + set_play + "__kicking_" + relation;
        if (modes.contains(advertised)) {
            return &modes[advertised];
        }
        if (modes.contains(legacy)) {
            return &modes[legacy];
        }
        if (modes.contains("normal_play")) {
            return &modes["normal_play"];
        }
        return nullptr;
    }

    // Resolve a plain scalar with a 3-level fallback: robot → mode defaults → formation defaults → fallback.
    inline double resolve_scalar(const std::string& key,
                                 const nlohmann::json& robot_cfg,
                                 const nlohmann::json& mode,
                                 const nlohmann::json& formation,
                                 double fallback) {
        if (robot_cfg.contains(key) && robot_cfg[key].is_number()) {
            return robot_cfg[key].get<double>();
        }
        if (mode.contains("defaults") && mode["defaults"].contains(key)
            && mode["defaults"][key].is_number()) {
            return mode["defaults"][key].get<double>();
        }
        if (formation.contains("defaults") && formation["defaults"].contains(key)
            && formation["defaults"][key].is_number()) {
            return formation["defaults"][key].get<double>();
        }
        return fallback;
    }

    // Resolve a nested scalar (e.g. attraction.x) with the same fallback chain.
    inline double resolve_nested_scalar(const std::string& parent,
                                        const std::string& child,
                                        const nlohmann::json& robot_cfg,
                                        const nlohmann::json& mode,
                                        const nlohmann::json& formation,
                                        double fallback) {
        auto try_get = [&](const nlohmann::json& obj) -> std::optional<double> {
            if (obj.contains(parent) && obj[parent].contains(child)
                && obj[parent][child].is_number()) {
                return obj[parent][child].get<double>();
            }
            return std::nullopt;
        };
        if (auto v = try_get(robot_cfg)) {
            return *v;
        }
        if (mode.contains("defaults")) {
            if (auto v = try_get(mode["defaults"])) {
                return *v;
            }
        }
        if (formation.contains("defaults")) {
            if (auto v = try_get(formation["defaults"])) {
                return *v;
            }
        }
        return fallback;
    }

    /// Compute where a support player should stand based on the formation JSON.
    /// Returns nullopt when: formation is empty, no mode resolves, or player is not in the mode.
    /// In all nullopt cases the caller should fall back to the Support module's own positioning.
    inline std::optional<Eigen::Vector2d> compute_support_position(
        const nlohmann::json& formation,
        int player_number,
        const std::string& game_phase,
        const std::string& state,
        const std::string& set_play,
        std::optional<int> kicking_team,
        int own_team_id,
        const Eigen::Vector2d& ball_field,
        const Dims& dims) {

        if (formation.empty()) {
            return std::nullopt;
        }
        const nlohmann::json* mode =
            resolve_mode(formation, game_phase, state, set_play, kicking_team, own_team_id);
        if (mode == nullptr) {
            return std::nullopt;
        }

        std::string pid = std::to_string(player_number);
        if (!mode->contains("robots") || !(*mode)["robots"].contains(pid)) {
            return std::nullopt;
        }
        const auto& robot_cfg = (*mode)["robots"][pid];

        nlohmann::json zero = 0.0;
        double offset_x =
            resolve_measure(robot_cfg.value("offset", nlohmann::json::object()).value("x", zero),
                            dims)
                .value_or(0.0);
        double offset_y =
            resolve_measure(robot_cfg.value("offset", nlohmann::json::object()).value("y", zero),
                            dims)
                .value_or(0.0);

        double attr_x = resolve_nested_scalar("attraction", "x", robot_cfg, *mode, formation, 1.0);
        double attr_y = resolve_nested_scalar("attraction", "y", robot_cfg, *mode, formation, 1.0);

        const double inf = std::numeric_limits<double>::infinity();
        double min_x     = resolve_scalar("minX", robot_cfg, *mode, formation, -inf);
        double max_x     = resolve_scalar("maxX", robot_cfg, *mode, formation, inf);
        double min_y     = resolve_scalar("minY", robot_cfg, *mode, formation, -inf);
        double max_y     = resolve_scalar("maxY", robot_cfg, *mode, formation, inf);

        return Eigen::Vector2d{std::clamp(ball_field.x() * attr_x + offset_x, min_x, max_x),
                               std::clamp(ball_field.y() * attr_y + offset_y, min_y, max_y)};
    }

}  // namespace utility::strategy::formation

#endif  // UTILITY_STRATEGY_FORMATION_HPP
