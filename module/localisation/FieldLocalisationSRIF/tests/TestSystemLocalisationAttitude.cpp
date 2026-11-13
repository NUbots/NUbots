/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <vector>

#include "measurement/MeasurementBodyRates.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/gaussian/GaussianInfo.hpp"
#include "utility/gaussian_filtering/rotation.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

using module::localisation::measurement::MeasurementBodyVelocity;
using module::localisation::measurement::MeasurementGyroscope;
using module::localisation::srif::SystemLocalisation;
using utility::gaussian_filtering::rpy2quat;
using utility::gaussian_filtering::gaussian::GaussianInfo;

namespace {

    /// @brief State vector from a pose in roll-pitch-yaw plus camera-mount bias
    Eigen::VectorXd make_state(double x,
                               double y,
                               double z,
                               double roll,
                               double pitch,
                               double yaw,
                               double bias_roll  = 0.0,
                               double bias_pitch = 0.0) {
        Eigen::VectorXd s = Eigen::VectorXd::Zero(SystemLocalisation::nx);
        s.head<3>() << x, y, z;
        s.segment<4>(SystemLocalisation::i_quat) = rpy2quat(Eigen::Vector3d(roll, pitch, yaw));
        s(SystemLocalisation::i_bias)            = bias_roll;
        s(SystemLocalisation::i_bias + 1)        = bias_pitch;
        return s;
    }

    GaussianInfo<double> tight_belief(const Eigen::VectorXd& mu, double std = 0.05) {
        const Eigen::MatrixXd S = Eigen::MatrixXd::Identity(SystemLocalisation::nx, SystemLocalisation::nx) * std;
        return GaussianInfo<double>::from_sqrt_moment(mu, S);
    }

    Eigen::VectorXd nominal_state() {
        return make_state(1.5, -0.8, 0.44, 0.0, 0.0, 0.3);
    }

    /// A state moving at a given body-fixed twist
    Eigen::VectorXd moving_state(const Eigen::Vector3d& v, const Eigen::Vector3d& w = Eigen::Vector3d::Zero()) {
        Eigen::VectorXd x                         = nominal_state();
        x.segment<3>(SystemLocalisation::i_vel)   = v;
        x.segment<3>(SystemLocalisation::i_omega) = w;
        return x;
    }

}  // namespace

SCENARIO("The quaternion attitude state round-trips through roll-pitch-yaw", "[slam][localisation]") {
    GIVEN("A state built from Euler angles") {
        const Eigen::VectorXd x = make_state(1.0, 2.0, 0.44, 0.1, -0.2, 1.3);

        THEN("attitude_rpy recovers them and heading reads the yaw") {
            const Eigen::Vector3d rpy = SystemLocalisation::attitude_rpy(x);
            REQUIRE_THAT(rpy(0), WithinAbs(0.1, 1e-12));
            REQUIRE_THAT(rpy(1), WithinAbs(-0.2, 1e-12));
            REQUIRE_THAT(rpy(2), WithinAbs(1.3, 1e-12));
            REQUIRE_THAT(SystemLocalisation::heading(x), WithinAbs(1.3, 1e-12));
        }

        THEN("the stored quaternion is a unit in the w >= 0 hemisphere") {
            const Eigen::Vector4d q = x.segment<4>(SystemLocalisation::i_quat);
            REQUIRE_THAT(q.norm(), WithinAbs(1.0, 1e-12));
            REQUIRE(q(0) >= 0.0);
        }
    }

    GIVEN("An attitude at the gimbal lock a forward fall passes through") {
        // Pitch = 90 deg is the singular point of a roll-pitch-yaw parameterisation
        const Eigen::VectorXd x = make_state(0.0, 0.0, 0.2, 0.0, 0.5 * M_PI, 0.7);

        THEN("the state is finite and its rotation matrix is orthonormal") {
            REQUIRE(x.allFinite());
            const Eigen::Matrix3d Rfb = SystemLocalisation::field_pose<double>(x).rotation_matrix;
            REQUIRE_THAT((Rfb.transpose() * Rfb - Eigen::Matrix3d::Identity()).norm(), WithinAbs(0.0, 1e-12));
        }
    }
}

SCENARIO("Attitude covariance maps the quaternion block onto the field axes", "[slam][localisation]") {
    GIVEN("An identity attitude with a known tangent-space uncertainty") {
        // dq = 0.5*Xi*dtheta, so a tangent std of s is a component std of s/2
        const Eigen::VectorXd x  = make_state(0.0, 0.0, 0.44, 0.0, 0.0, 0.0);
        const double sigma_roll  = 0.03;
        const double sigma_pitch = 0.07;
        const double sigma_yaw   = 0.11;
        const Eigen::Matrix3d Sig =
            Eigen::Vector3d(sigma_roll * sigma_roll, sigma_pitch * sigma_pitch, sigma_yaw * sigma_yaw).asDiagonal();

        const Eigen::Matrix<double, 4, 3> J = SystemLocalisation::attitude_tangent_field(x);
        Eigen::MatrixXd P                   = Eigen::MatrixXd::Zero(SystemLocalisation::nx, SystemLocalisation::nx);
        P.block<4, 4>(SystemLocalisation::i_quat, SystemLocalisation::i_quat) = J * Sig * J.transpose();

        THEN("the reported per-axis std devs are exactly what went in") {
            // The inverse must not apply attitude_tangent's 0.5 a second time
            const Eigen::Vector3d s = SystemLocalisation::attitude_std(x, P);
            REQUIRE_THAT(s(0), WithinRel(sigma_roll, 1e-9));
            REQUIRE_THAT(s(1), WithinRel(sigma_pitch, 1e-9));
            REQUIRE_THAT(s(2), WithinRel(sigma_yaw, 1e-9));
            REQUIRE_THAT(std::sqrt(SystemLocalisation::yaw_variance(x, P)), WithinRel(sigma_yaw, 1e-9));
        }
    }

    GIVEN("An arbitrary attitude") {
        const Eigen::VectorXd x = make_state(0.0, 0.0, 0.44, 0.4, -0.9, 2.1);

        THEN("attitude_jacobian is a left inverse of attitude_tangent_field") {
            const Eigen::Matrix<double, 3, 4> G = SystemLocalisation::attitude_jacobian(x);
            const Eigen::Matrix<double, 4, 3> J = SystemLocalisation::attitude_tangent_field(x);
            REQUIRE_THAT((G * J - Eigen::Matrix3d::Identity()).norm(), WithinAbs(0.0, 1e-12));
        }
    }
}

SCENARIO("Recovery from a fall widens the belief without moving it", "[slam][localisation]") {
    GIVEN("A confident single-hypothesis belief") {
        const Eigen::VectorXd x0 = nominal_state();
        SystemLocalisation sys(tight_belief(x0));
        sys.reset_to(tight_belief(x0), 0.0);
        const Eigen::MatrixXd P0 = sys.density.cov();

        WHEN("the recovery inflation is applied") {
            // Built as FieldLocalisationSRIF builds it: position on the diagonal, yaw as a
            // rank-one block about the field z axis
            const double yaw_std  = 1.0;
            Eigen::MatrixXd extra = Eigen::MatrixXd::Zero(SystemLocalisation::nx, SystemLocalisation::nx);
            extra(0, 0) = extra(1, 1)   = 0.25;  // 0.5 m std
            const Eigen::Vector4d j_yaw = SystemLocalisation::attitude_tangent_field(x0).col(2);
            extra.block<4, 4>(SystemLocalisation::i_quat, SystemLocalisation::i_quat) =
                yaw_std * yaw_std * j_yaw * j_yaw.transpose();
            const double yaw_var0 = SystemLocalisation::yaw_variance(x0, P0);
            sys.inflate_covariance(extra);

            THEN("the mean is untouched") {
                // The pre-fall position is still the best estimate
                REQUIRE_THAT((sys.density.mean() - x0).norm(), WithinAbs(0.0, 1e-9));
            }

            THEN("exactly the requested yaw variance is added") {
                // The reported yaw uncertainty grows by exactly what was asked for
                const Eigen::MatrixXd P = sys.density.cov();
                REQUIRE_THAT(P(0, 0) - P0(0, 0), WithinAbs(0.25, 1e-9));
                REQUIRE_THAT(SystemLocalisation::yaw_variance(sys.density.mean(), P) - yaw_var0,
                             WithinRel(yaw_std * yaw_std, 1e-6));
            }

            THEN("roll and pitch are left alone") {
                // A yaw inflation must not spread into the other two attitude axes
                const Eigen::Matrix3d A0 = SystemLocalisation::attitude_covariance(x0, P0);
                const Eigen::Matrix3d A =
                    SystemLocalisation::attitude_covariance(sys.density.mean(), sys.density.cov());
                REQUIRE_THAT(A(0, 0), WithinRel(A0(0, 0), 1e-6));
                REQUIRE_THAT(A(1, 1), WithinRel(A0(1, 1), 1e-6));
            }

            THEN("the camera mount bias keeps its confidence") {
                // The camera bias does not depend on posture
                const Eigen::MatrixXd P = sys.density.cov();
                REQUIRE_THAT(P(SystemLocalisation::i_bias, SystemLocalisation::i_bias),
                             WithinRel(P0(SystemLocalisation::i_bias, SystemLocalisation::i_bias), 1e-9));
            }
        }
    }

    GIVEN("An active hypothesis bank") {
        const Eigen::VectorXd x0 = nominal_state();
        SystemLocalisation sys(tight_belief(x0));
        sys.reset_to(tight_belief(x0), 0.0);
        sys.initialise_hypotheses();
        REQUIRE(sys.num_hypotheses() == 2);

        WHEN("the recovery inflation is applied") {
            Eigen::VectorXd extra = Eigen::VectorXd::Zero(SystemLocalisation::nx);
            extra(0) = extra(1) = 0.25;
            sys.inflate_covariance(extra);

            THEN("every hypothesis is widened, not just the representative") {
                REQUIRE(sys.num_hypotheses() == 2);
                for (const GaussianInfo<double>& c : sys.hypotheses()) {
                    REQUIRE(std::sqrt(c.cov()(0, 0)) > 0.5);
                }
            }
        }
    }
}

SCENARIO("The velocity states carry the estimate, and measurements steer them", "[slam][localisation]") {
    GIVEN("A belief moving at 0.3 m/s along the body x axis") {
        const Eigen::VectorXd x0 = moving_state(Eigen::Vector3d(0.3, 0.0, 0.0));

        WHEN("two seconds are predicted") {
            SystemLocalisation sys(tight_belief(x0));
            sys.reset_to(tight_belief(x0), 0.0);
            sys.predict_all(2.0);
            const Eigen::VectorXd mu = sys.density.mean();

            THEN("the velocity state carries the position forward") {
                // 0.3 m/s for 2 s along body x, rotated into {f} by yaw 0.3
                REQUIRE_THAT(mu(0), WithinAbs(x0(0) + 0.6 * std::cos(0.3), 0.05));
                REQUIRE_THAT(mu(1), WithinAbs(x0(1) + 0.6 * std::sin(0.3), 0.05));
            }
        }

        WHEN("a zero-velocity update is applied") {
            SystemLocalisation sys(tight_belief(x0));
            sys.reset_to(tight_belief(x0), 0.0);
            MeasurementBodyVelocity zupt = MeasurementBodyVelocity::stationary(0.0, 0.01);
            sys.process(zupt);

            THEN("the velocity state is pulled to a stop") {
                REQUIRE(SystemLocalisation::body_velocity(sys.density.mean()).norm() < 0.05);
            }

            THEN("and the estimate then stays put under prediction") {
                // This keeps a fallen robot on the spot
                const Eigen::VectorXd before = sys.density.mean();
                sys.predict_all(4.0);
                REQUIRE((sys.density.mean().head<2>() - before.head<2>()).norm() < 0.05);
            }
        }
    }

    GIVEN("A belief with a spin about the body z axis") {
        const Eigen::VectorXd x0 = moving_state(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.5));

        WHEN("one second is predicted") {
            SystemLocalisation sys(tight_belief(x0));
            sys.reset_to(tight_belief(x0), 0.0);
            sys.predict_all(1.0);

            THEN("the heading follows the angular velocity state") {
                REQUIRE_THAT(SystemLocalisation::heading(sys.density.mean()),
                             WithinAbs(SystemLocalisation::heading(x0) + 0.5, 0.05));
            }
        }
    }
}

SCENARIO("The gyroscope measures the rate, and its bias is observable", "[slam][localisation]") {
    GIVEN("A stationary belief and a gyroscope reading a pure bias") {
        // A robot held still while the gyro reads 0.02 rad/s about z. The reading is
        // omegaBb + bGyro, so a zero-velocity update stands in for the landmark measurements
        // that separate the two.
        const Eigen::VectorXd x0 = nominal_state();
        SystemLocalisation sys(tight_belief(x0, 0.02));
        sys.reset_to(tight_belief(x0, 0.02), 0.0);

        WHEN("the gyroscope reading is applied with the rate held at zero") {
            for (int i = 1; i <= 200; ++i) {
                const double t = 0.01 * i;
                MeasurementGyroscope gyro(t, Eigen::Vector3d(0.0, 0.0, 0.02), 0.02);
                sys.process(gyro);
                MeasurementBodyVelocity zupt = MeasurementBodyVelocity::stationary(t, 0.01);
                sys.process(zupt);
            }

            THEN("the reading is explained by the rate and bias together") {
                const Eigen::VectorXd mu = sys.density.mean();
                const double predicted = SystemLocalisation::body_rate(mu).z() + SystemLocalisation::gyro_bias(mu).z();
                REQUIRE_THAT(predicted, WithinAbs(0.02, 0.004));
            }
        }
    }
}

SCENARIO("Prediction through a topple tracks the rotation rather than a chart of it", "[slam][localisation]") {
    // Pitching through 90 deg is an ordinary point for a quaternion. attitude_rpy() reads a
    // roll-pitch-yaw chart that is discontinuous there, so continuity is tested on the
    // rotation itself.
    GIVEN("A robot pitching over at 1.5 rad/s from upright") {
        const Eigen::VectorXd x0 = make_state(0.0, 0.0, 0.44, 0.0, 0.0, 1.0);
        const double rate        = 1.5;
        Eigen::VectorXd xr       = x0;
        xr.segment<3>(SystemLocalisation::i_omega) << 0.0, rate, 0.0;

        SystemLocalisation sys(tight_belief(xr));
        sys.reset_to(tight_belief(xr), 0.0);

        WHEN("it is predicted in small steps through and past vertical") {
            // A whole revolution about the body y axis, stepped as exact fractions of it so
            // the final step lands on the revolution
            const double revolution  = 2.0 * M_PI / rate;
            constexpr int steps      = 84;  // ~0.05 s apart
            const double step        = revolution / steps;
            bool finite              = true;
            double max_step_rotation = 0.0;
            Eigen::Matrix3d previous = SystemLocalisation::field_pose<double>(xr).rotation_matrix;

            for (int i = 1; i <= steps; ++i) {
                const double t = i * step;
                sys.predict_all(t);
                const Eigen::VectorXd mu = sys.density.mean();
                finite                   = finite && mu.allFinite() && sys.density.cov().allFinite();

                // The angle of the incremental rotation between consecutive steps
                const Eigen::Matrix3d Rfb = SystemLocalisation::field_pose<double>(mu).rotation_matrix;
                const double c            = 0.5 * ((previous.transpose() * Rfb).trace() - 1.0);
                max_step_rotation         = std::max(max_step_rotation, std::acos(std::clamp(c, -1.0, 1.0)));
                previous                  = Rfb;
            }

            THEN("the state never goes non-finite") {
                REQUIRE(finite);
            }

            THEN("the rotation is continuous the whole way round") {
                // No jump near the half revolution a gimbal alias would introduce
                REQUIRE(max_step_rotation < 1.5 * rate * step);
            }

            THEN("the attitude stays on the unit sphere") {
                const Eigen::Vector4d q = sys.density.mean().segment<4>(SystemLocalisation::i_quat);
                REQUIRE_THAT(q.norm(), WithinAbs(1.0, 1e-6));
            }

            THEN("a full revolution returns the robot upright on its original heading") {
                // The robot ends upright on the heading it started with
                const Eigen::Vector3d rpy = SystemLocalisation::attitude_rpy(sys.density.mean());
                REQUIRE_THAT(rpy(0), WithinAbs(0.0, 1e-3));
                REQUIRE_THAT(rpy(1), WithinAbs(0.0, 1e-3));
                REQUIRE_THAT(std::remainder(rpy(2) - 1.0, 2.0 * M_PI), WithinAbs(0.0, 1e-3));
            }
        }
    }
}

SCENARIO("The field mirror is an exact linear map on the state", "[slam][localisation]") {
    GIVEN("A pose and its 180 degree mirror") {
        const Eigen::VectorXd x = make_state(1.5, -0.8, 0.44, 0.05, -0.1, 0.3, 0.01, -0.02);
        const Eigen::VectorXd m = SystemLocalisation::mirror_state(x);

        THEN("the horizontal position is negated and the height and bias are untouched") {
            REQUIRE_THAT(m(0), WithinAbs(-x(0), 1e-12));
            REQUIRE_THAT(m(1), WithinAbs(-x(1), 1e-12));
            REQUIRE_THAT(m(2), WithinAbs(x(2), 1e-12));
            REQUIRE_THAT(m(SystemLocalisation::i_bias), WithinAbs(x(SystemLocalisation::i_bias), 1e-12));
            REQUIRE_THAT(m(SystemLocalisation::i_bias + 1), WithinAbs(x(SystemLocalisation::i_bias + 1), 1e-12));
        }

        THEN("the heading is turned by pi and roll/pitch are preserved") {
            const Eigen::Vector3d rx = SystemLocalisation::attitude_rpy(x);
            const Eigen::Vector3d rm = SystemLocalisation::attitude_rpy(m);
            REQUIRE_THAT(std::remainder(rm(2) - rx(2) - M_PI, 2.0 * M_PI), WithinAbs(0.0, 1e-12));
            REQUIRE_THAT(rm(0), WithinAbs(rx(0), 1e-12));
            REQUIRE_THAT(rm(1), WithinAbs(rx(1), 1e-12));
        }

        THEN("mirroring twice is the identity on the pose") {
            const Eigen::VectorXd back = SystemLocalisation::mirror_state(m);
            REQUIRE_THAT((back.head<3>() - x.head<3>()).norm(), WithinAbs(0.0, 1e-12));
            REQUIRE_THAT((SystemLocalisation::attitude_rpy(back) - SystemLocalisation::attitude_rpy(x)).norm(),
                         WithinAbs(0.0, 1e-12));
        }
    }

    GIVEN("A density and its mirror") {
        const Eigen::VectorXd x       = nominal_state();
        const GaussianInfo<double> g  = tight_belief(x, 0.07);
        const GaussianInfo<double> gm = SystemLocalisation::mirror_density(g);

        THEN("the reported yaw uncertainty is unchanged") {
            // mirror_quat_map is orthogonal, so P' = M*P*M^T is exact
            REQUIRE_THAT(SystemLocalisation::yaw_variance(gm.mean(), gm.cov()),
                         WithinRel(SystemLocalisation::yaw_variance(g.mean(), g.cov()), 1e-6));
        }
    }
}

SCENARIO("Every state takes process noise", "[slam][localisation]") {
    GIVEN("A constructed estimator") {
        SystemLocalisation sys(tight_belief(nominal_state()));

        THEN("process_noise_index covers the whole state") {
            // The list must match process_noise_density's dimension
            REQUIRE(sys.process_noise_index().size() == std::size_t(SystemLocalisation::nx));
            REQUIRE(sys.process_noise_density(0.1).dim() == SystemLocalisation::nx);
        }
    }
}
