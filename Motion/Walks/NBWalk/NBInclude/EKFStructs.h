#ifndef EKFStructs_h
#define EKFStructs_h
#include <ostream>

// Structs
// Odometery change
class MotionModel
{
public:
    MotionModel(float f = 0.0f, float l = 0.0f, float r = 0.0f)
        : deltaF(f), deltaL(l), deltaR(r) { }
    MotionModel(const MotionModel& other)
        : deltaF(other.deltaF), deltaL(other.deltaL), deltaR(other.deltaR) { }
    float deltaF;
    float deltaL;
    float deltaR;

    friend std::ostream& operator<< (std::ostream &o, const MotionModel &u) {
        return o << "(" << u.deltaF << ", " << u.deltaL << ", " << u.deltaR
                 << ")";
    }

};

/**
 * AccelMeasurement - A non-generic class for holding accelerometer values
 *                    required (by motion) for filtering accel sensor values.
 */
struct AccelMeasurement {
    float x;
    float y;
    float z;
};

/**
 * AngleMeasurement - a class for holding angleX, angleY
 */

struct AngleMeasurement{
  float angleX;
  float angleY;
};


/**
 * ZmpTimeUpdate - For ZmpEKF, the apriori input comes as the zmp from the
 *                 third entry in the controller's state vector.
 *
 */

struct ZmpTimeUpdate{
    float cur_zmp_x;
    float cur_zmp_y;
};

/**
 * AccelMeasurement - A non-generic class for holding accelerometer values
 *                    required (by motion) for filtering accel sensor values.
 */
struct ZmpMeasurement {
    float comX;
    float comY;
    float accX;
    float accY;
};

#endif
