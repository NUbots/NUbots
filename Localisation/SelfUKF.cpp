#include "SelfUKF.h"

/*! @brief Default constructor
 */
SelfUKF::SelfUKF(): SelfModel(0.0)
{
}

/*! @brief Default time constructor

    This constructor requires a creation time.

 */
SelfUKF::SelfUKF(double time): SelfModel(time)
{
}

/*! @brief Copy constructor

 */
SelfUKF::SelfUKF(const SelfUKF& source): SelfModel(source)
{
}

/*! @brief Split constructor

Takes a parent filter and performs a split on an ambiguous object using the given split option.

@param parent The parent model from which the split orignates.
@param object The ambiguous object belong evaluated by the split.
@param splitOption The option to be evaluated within this model.
@param time The current time of the update
*/
SelfUKF::SelfUKF(const SelfUKF& parent, const AmbiguousObject& object, const StationaryObject& splitOption, float time):
        SelfModel(parent, object, splitOption, time)
{

}

/*! @brief Time update
Performs a time update. The time update uses the oboemtry estimate and time information to
run the predictive stage of the kalman filter.

@param odometry The odmetry is represented by a three valued vector.
@param deltaTime The change in time elapsed since the previous time update.
*/
void SelfUKF::TimeUpdate(const std::vector<float>& odometry, float deltaTime)
{
    float x = odometry[0];
    float y = odometry[1];
    float heading = odometry[2];

    return;
}

void MeasurementUpdate(const std::vector<StationaryObject>& objects)
{
    for (std::vector<StationaryObject>::const_iterator it = objects.begin(); it != objects.end(); it++)
    {

    }
}
