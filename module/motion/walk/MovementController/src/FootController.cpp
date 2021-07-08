#include "FootController.hpp"

namespace module {
    namespace motion {
        namespace walk {

            // Returns x-position of vector field. See vectorfield.py for graphical representation of the vector field
            double FootController::f_x(const Eigen::Vector3d& pos) {
                // Prevent divide by zero error with 0 position
                return tanh(pos.x() * config.scaling_factor);
            }

            // Returns z-position of vector field. See vectorfield.py for graphical representation of the vector field
            double FootController::f_z(const Eigen::Vector3d& pos) {
                return -(config.step_height * pow(tanh(pos.x() * config.scaling_factor), 2) - pos.z())
                       * config.scaling_factor;
            }

            double integral(double x, double h, double c) {
                return sqrt(1 + pow(cosh(x) * (c + h * atan(1 / sinh(x))) - h * tanh(x), 2));
            }

            // path length to origin; x,y=foots starting location; h=step height; -1<x<1 0<y<1 0<h<1
            double FootController::pathlength(const Eigen::Vector3d& pos) {
                double x      = config.scaling_factor * pos.x();
                double y      = config.scaling_factor * pos.z();
                double h      = config.scaling_factor * config.step_height;
                double c      = y / sinh(x) - h * atan(1 / sinh(x));
                int num_steps = 32;

                double x2   = x / num_steps;
                double x1   = 0;
                double path = 0;

                int i = 0;
                while (i < num_steps) {
                    path = path
                           + ((x2 - x1) / 6)
                                 * (integral(x1, h, c) + 4 * (integral((x1 + x2) / 2, h, c)) + integral(x2, h, c));
                    x1 = x2;
                    x2 = x2 + x / num_steps;
                    i++;
                }

                return path / config.scaling_factor;
            }

            /**
             * Calculates the next swing foot position we should be at in time_horizon amount of time
             *
             * @param time_horizon Time into the future with which we project the next position to
             * @param time_left The amount of time remaining until we should hit our target
             * @param Hwg Homogeneous transform from ground space (g) to current s"wing" (w) foot space
             * @param Hw_tg Homogeneous transform from ground space (g) to s"wing" target (t) foot space
             *
             * @return Hng Homogeneous transform from ground space to next target s"wing" (n) foot space
             */
            Eigen::Affine3d FootController::next_swing(const double& time_horizon,
                                                       const double& time_left,
                                                       const Eigen::Affine3d& Hwg,
                                                       const Eigen::Affine3d& Htg) {
                double factor = time_horizon / time_left;


                //---------------------------------------------------------------
                //-------------VECTOR FIELD--------------------------------------
                //---------------------------------------------------------------

                // Get the vector from target to ground in ground space
                // Htg.rotation().transpose() = Rgt
                // Htg.translation() = rGTt
                // -rGTt = rTGt
                // Rgt * rGTt = rGTg
                const Eigen::Vector3d rTGg = -Htg.rotation().transpose() * Htg.translation();

                // Get the vector from ground to wing foot in ground space
                // Hwg.rotation().transpose() = Rgw
                // Hwg.translation() = rGWw
                // -rGWw = rWGw
                // Rgw * rWGw = rWGg
                const Eigen::Vector3d rWGg = -Hwg.rotation().transpose() * Hwg.translation();

                // Get the vector from target to wing foot in ground space
                const Eigen::Vector3d rWTg = rWGg - rTGg;


                // Use the vector field to get the wing target to next target position
                // The vector field gives us the swing foot to next position vector in ground space (rNWg)
                // rNWg + rWTg = rNTg
                // We normalize the vector and multiply it by the distance and factor to reach the target at the right
                // time
                Eigen::Vector3d rNWg =
                    Eigen::Vector3d(f_x(rWTg), 0, f_z(rWTg)).normalized() * factor * pathlength(rWTg);
                Eigen::Vector3d rNTg = rWTg + rNWg;

                if (rWTg.z() + rNWg.z() < 0) {
                    rNTg.z() = 0;
                }

                // Vector field does not take into account the y-value so we linearly interpolate so it will reach the
                // target
                rNTg.y() = rWTg.y() * (1 - factor);

                // Retrieve our final vector for the translation component of Hgn
                Eigen::Vector3d rNGg = rNTg + rTGg;

                //---------------------------------------------------------------
                //---------------------------------------------------------------

                // Get the distance to the target
                double distance = rWTg.norm();

                // Create the Hgn matrix
                // Slerp the rotation
                // Set the translation
                Eigen::Affine3d Hgn;

                Hgn.linear() = Eigen::Quaterniond(Htg.rotation())
                                   .slerp(factor, Eigen::Quaterniond(Hwg.rotation()))
                                   .toRotationMatrix()
                                   .transpose();

                // If we are very close to the target, just go to the target directly
                // if (rWTg.norm() < 0.025) {
                //    rNGg = rTGg;
                //}
                if ((Htg.rotation() - Hwg.rotation()).maxCoeff() < 0.001 || factor == 1) {
                    Hgn.linear() = Htg.rotation().transpose();
                }

                Hgn.translation() = rNGg;

                return Hgn.inverse();
            }

        }  // namespace walk
    }      // namespace motion
}  // namespace module
