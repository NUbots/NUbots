#ifndef SELFUKF_H
#define SELFUKF_H
#include "SelfModel.h"
#include "Infrastructure/FieldObjects/StationaryObject.h"

class SelfUKF: public SelfModel
{
public:
    // Constructors
    SelfUKF();
    SelfUKF(double time);
    SelfUKF(const SelfUKF& source);
    SelfUKF(const SelfUKF& parent, const AmbiguousObject& object, const StationaryObject& splitOption, float time);

    // Update functions
    void TimeUpdate(const std::vector<float>& odometry, float deltaTime);
    void MeasurementUpdate(const std::vector<StationaryObject>& objects);

};

#endif // SELFUKF_H
