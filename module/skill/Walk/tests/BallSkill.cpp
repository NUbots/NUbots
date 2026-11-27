/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
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
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "ball_skill.hpp"
#include "ball_skill_policy.hpp"

using Catch::Approx;
using namespace module::skill::ball_skill;

namespace {

    Frame frame_of(float v) {
        Frame f{};
        f.gyro.fill(v);
        f.gravity.fill(v);
        f.q.fill(v);
        f.dq.fill(v);
        f.action.fill(v);
        f.command.fill(v);
        f.ball.fill(v);
        return f;
    }

}  // namespace

TEST_CASE("The ball-skill input is 234 numbers", "[BallSkill]") {
    REQUIRE(INPUT_SIZE == 234);
}

TEST_CASE("The first step fills the history, later steps shift it, oldest first", "[BallSkill]") {
    History history;
    history.push(frame_of(1.0f));
    auto obs = history.input();
    REQUIRE(obs.size() == INPUT_SIZE);
    for (const float v : obs) {
        REQUIRE(v == 1.0f);
    }

    history.push(frame_of(2.0f));
    history.push(frame_of(3.0f));
    obs = history.input();
    // gyro term: steps 1, 2, 3 (three numbers each), then gravity's
    REQUIRE(obs[0] == 1.0f);
    REQUIRE(obs[3] == 2.0f);
    REQUIRE(obs[6] == 3.0f);
    REQUIRE(obs[9] == 1.0f);
    // the ball term is last: 4 numbers per step
    REQUIRE(obs[INPUT_SIZE - 12] == 1.0f);
    REQUIRE(obs[INPUT_SIZE - 8] == 2.0f);
    REQUIRE(obs[INPUT_SIZE - 4] == 3.0f);
    // the command term sits before the ball: 2 numbers per step
    REQUIRE(obs[INPUT_SIZE - 12 - 6] == 1.0f);
    REQUIRE(obs[INPUT_SIZE - 12 - 2] == 3.0f);

    history.push(frame_of(4.0f));
    obs = history.input();
    REQUIRE(obs[0] == 2.0f);
    REQUIRE(obs[6] == 4.0f);

    history.reset();
    history.push(frame_of(5.0f));
    REQUIRE(history.input()[0] == 5.0f);
}

TEST_CASE("A world point lands in the policies' camera frame", "[BallSkill]") {
    Camera camera;
    camera.Hpc    = Eigen::Isometry3d::Identity();
    camera.offset = Eigen::Vector3d(0.06, 0.0, 0.07);

    // The camera {c} sits 0.5 m above the world origin, axes aligned with the world
    Eigen::Isometry3d Hwc   = Eigen::Isometry3d::Identity();
    Hwc.translation()       = Eigen::Vector3d(0.0, 0.0, 0.5);
    const Eigen::Vector3d r = to_policy_camera(Eigen::Vector3d(1.0, 0.2, 0.0), Hwc.inverse(), camera);
    REQUIRE(r.x() == Approx(0.94));
    REQUIRE(r.y() == Approx(0.2));
    REQUIRE(r.z() == Approx(-0.57));

    // With the camera mounted pitched down on the head pitch link, Hpc rotates it back into {p}
    camera.Hpc.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d rBCc(1.0, 0.0, 0.0);
    const Eigen::Vector3d expected = camera.Hpc.linear() * rBCc - camera.offset;
    REQUIRE((to_policy_camera(rBCc, Eigen::Isometry3d::Identity(), camera) - expected).norm() < 1e-12);
}

TEST_CASE("The field of view and range cut the ball observation", "[BallSkill]") {
    const Camera camera;
    REQUIRE(in_view(Eigen::Vector3d(1.0, 0.0, -0.5), camera));
    REQUIRE_FALSE(in_view(Eigen::Vector3d(-1.0, 0.0, 0.0), camera));
    // 106.2 degrees wide: 60 degrees off axis is out
    REQUIRE_FALSE(in_view(Eigen::Vector3d(std::cos(1.05), std::sin(1.05), 0.0), camera));
    REQUIRE(in_view(Eigen::Vector3d(std::cos(0.9), std::sin(0.9), 0.0), camera));
    // 95 degrees tall: 50 degrees down is out
    REQUIRE_FALSE(in_view(Eigen::Vector3d(1.0, 0.0, -std::tan(0.873)), camera));
    REQUIRE_FALSE(in_view(Eigen::Vector3d(9.5, 0.0, 0.0), camera));
}

TEST_CASE("A detection ray is put on the ball centre", "[BallSkill]") {
    Eigen::Isometry3d Hwc       = Eigen::Isometry3d::Identity();
    Hwc.translation()           = Eigen::Vector3d(0.0, 0.0, 1.0);
    const Eigen::Isometry3d Hcw = Hwc.inverse();

    const auto rBWw = ray_to_ball_centre(Eigen::Vector3d(1.0, 0.0, -1.0), Hcw, 0.1);
    REQUIRE(rBWw.has_value());
    REQUIRE(rBWw->x() == Approx(0.9));
    REQUIRE(rBWw->y() == Approx(0.0).margin(1e-12));
    REQUIRE(rBWw->z() == Approx(0.1));

    // Above the horizon the ray never reaches the ball's plane
    REQUIRE_FALSE(ray_to_ball_centre(Eigen::Vector3d(1.0, 0.0, 0.1), Hcw, 0.1).has_value());
}

TEST_CASE("Commands are clamped to the trained ranges", "[BallSkill]") {
    const CommandLimits limits;
    REQUIRE(clamp_command(Kind::RECEIVE, Eigen::Vector2d(1.0, 1.0), limits).isZero());

    // A dribble under half the minimum is a hold, otherwise the speed is kept within 0.4 to 1.8
    REQUIRE(clamp_command(Kind::DRIBBLE, Eigen::Vector2d(0.1, 0.0), limits).isZero());
    REQUIRE(clamp_command(Kind::DRIBBLE, Eigen::Vector2d(0.3, 0.0), limits).x() == Approx(0.4));
    const Eigen::Vector2d fast = clamp_command(Kind::DRIBBLE, Eigen::Vector2d(0.0, -2.5), limits);
    REQUIRE(fast.norm() == Approx(1.8));
    REQUIRE(fast.y() == Approx(-1.8));

    // A shot over 4.5 m/s is kept within 45 degrees of the heading
    const Eigen::Vector2d shot = clamp_command(Kind::KICK, Eigen::Vector2d(0.0, 8.0), limits);
    REQUIRE(shot.norm() == Approx(7.0));
    REQUIRE(std::atan2(shot.y(), shot.x()) == Approx(M_PI / 4.0));
    // A pass within 135 degrees, and at least 2 m/s
    const Eigen::Vector2d pass = clamp_command(Kind::KICK_AIR, Eigen::Vector2d(std::cos(3.0), std::sin(3.0)), limits);
    REQUIRE(pass.norm() == Approx(2.0));
    REQUIRE(std::atan2(pass.y(), pass.x()) == Approx(3.0 * M_PI / 4.0));
    // No direction at all: straight ahead
    REQUIRE(clamp_command(Kind::KICK, Eigen::Vector2d::Zero(), limits).x() == Approx(2.0));

    REQUIRE(clamp_command(Kind::FIRST_TOUCH, Eigen::Vector2d(0.0, 3.0), limits).y() == Approx(1.2));
    REQUIRE(clamp_command(Kind::FIRST_PASS, Eigen::Vector2d(-1.0, 0.0), limits).x() == Approx(-2.0));
}

TEST_CASE("Joint targets convert to and from a policy's raw action", "[BallSkill]") {
    const double default_pose = 0.4;
    const double scale        = 0.464;
    const float action        = to_action(0.6, default_pose, scale);
    REQUIRE(default_pose + scale * action == Approx(0.6));
    REQUIRE(to_action(1.0, 0.0, 0.0) == 0.0f);
}

TEST_CASE("The head target is rate limited", "[BallSkill]") {
    REQUIRE(rate_limit(0.0, 1.0, 0.157) == Approx(0.157));
    REQUIRE(rate_limit(0.0, -1.0, 0.157) == Approx(-0.157));
    REQUIRE(rate_limit(0.5, 0.55, 0.157) == Approx(0.55));
}

TEST_CASE("mjlab joint names match JointIndexK1", "[BallSkill]") {
    REQUIRE(canonical("AAHead_yaw") == "head_yaw");
    REQUIRE(canonical("ALeft_Shoulder_Pitch") == "left_shoulder_pitch");
    REQUIRE(canonical("Right_Ankle_Roll") == "right_ankle_roll");
    REQUIRE(canonical("Head_Yaw") == JOINT_NAMES[0]);
}
