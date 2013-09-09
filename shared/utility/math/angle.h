#ifndef UTILITY_MATH_ANGLE_H
#define UTILITY_MATH_ANGLE_H
namespace utility {
namespace math {
namespace angle {
    constexpr double normalizeAngle(double angleInRadians) {
        return 
            fmod(angleInRadians + M_PI, M_PI * 2) 
            + fmod(angleInRadians + M_PI, M_PI * 2) < 0 ? M_PI * 2 : -M_PI;
    }
}
}
}
#endif
