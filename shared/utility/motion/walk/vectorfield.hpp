#include <Eigen/Core>

namespace utility::motion::walk {
    // Returns x-position of vector field. See module/motion/walk/docs/vectorfield.py for graphical representation of
    // the vector field
    template <typename Scalar>
    Scalar f_x(const Eigen::Matrix<Scalar, 3, 1>& pos, Scalar scaling_factor) {
        // Prevent divide by zero error with 0 position
        return tanh(pos.x() * scaling_factor);
    }

    // Returns z-position of vector field. See vectorfield.py for graphical representation of the vector field
    template <typename Scalar>
    Scalar f_z(const Eigen::Matrix<Scalar, 3, 1>& pos, const Scalar step_height, const Scalar scaling_factor) {
        return -(step_height * pow(tanh(pos.x() * scaling_factor), 2) - pos.z()) * scaling_factor;
    }

    // Integral for the vector field used in calculating the path length
    template <typename Scalar>
    Scalar integral(Scalar x, Scalar h, Scalar c) {
        return sqrt(1 + pow(cosh(x) * (c + h * atan(1 / sinh(x))) - h * tanh(x), 2));
    }

    // Calculates the length of the path of the vector field from the current foot position to the origin.
    template <typename Scalar>
    Scalar pathlength(const Eigen::Matrix<Scalar, 3, 1>& pos,
                      const int num_steps,
                      const Scalar scaling_factor,
                      const Scalar step_height) {
        Scalar x = scaling_factor * pos.x();
        Scalar y = scaling_factor * pos.z();
        Scalar h = scaling_factor * step_height;
        Scalar c = y / sinh(x) - h * atan(1 / sinh(x));

        Scalar x2   = x / num_steps;
        Scalar x1   = 0;
        Scalar path = 0;

        int i = 0;
        while (i < num_steps) {
            path = path
                   + ((x2 - x1) / 6) * (integral(x1, h, c) + 4 * (integral((x1 + x2) / 2, h, c)) + integral(x2, h, c));
            x1 = x2;
            x2 = x2 + x / num_steps;
            i++;
        }

        return path / scaling_factor;
    }

}  // namespace utility::motion::walk
