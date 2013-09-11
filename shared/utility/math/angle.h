#ifndef UTILITY_MATH_ANGLE_H
#define UTILITY_MATH_ANGLE_H

#include <cmath>

namespace utility {
namespace math {
namespace angle {
    double normalizeAngle(const double value) {

        double angle = fmod(value, 2 * M_PI);
        if (angle < -M_PI) angle += 2 * M_PI;
        else if (angle >= M_PI) angle -= 2 * M_PI;

        return angle;
    }
}
}
}
#endif
