#include <Eigen/Core>

namespace utility::motion::walk {
    // Returns x-position of vector field. See vectorfield.py for graphical representation of the vector field
    double f_x(const Eigen::Vector3d& pos, float scaling_factor) {
        // Prevent divide by zero error with 0 position
        return tanh(pos.x() * scaling_factor);
    }

    // Returns z-position of vector field. See vectorfield.py for graphical representation of the vector field
    double f_z(const Eigen::Vector3d& pos, const float step_height, const float scaling_factor) {
        return -(step_height * pow(tanh(pos.x() * scaling_factor), 2) - pos.z())
                * scaling_factor;
    }

    double integral(double x, double h, double c) {
        return sqrt(1 + pow(cosh(x) * (c + h * atan(1 / sinh(x))) - h * tanh(x), 2));
    }

    // path length to origin; x,y=foots starting location; h=step height; -1<x<1 0<y<1 0<h<1
    double pathlength(const Eigen::Vector3d& pos, const int num_steps, const float scaling_factor, const float step_height) {
        double x      = scaling_factor * pos.x();
        double y      = scaling_factor * pos.z();
        double h      = scaling_factor * step_height;
        double c      = y / sinh(x) - h * atan(1 / sinh(x));

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

        return path / scaling_factor;
    }

}
