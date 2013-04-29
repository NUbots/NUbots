#include "CM730.h"
#include "SensorReadDescriptor.h"

using namespace Robot;

// Update a response rate estimate given an error value.
// It doesn't matter how this is done, so long as it's relatively fast
// and results in reasonable performance.
double SensorReadDescriptor::UpdateResponseRate(int error_code)
{
    double old_rate = response_rate_;
    double new_value = (error_code == Robot::CM730::SUCCESS)? 1.0 : 0.0;
    // double new_value = (error_code != -1)? 1.0 : 0.0; // allows errors
    
    // This value is arbitrary, and should be tuned for reasonable performance
    double old_factor = 0.75;
    double new_factor = 1 - old_factor;
    double new_rate = (old_factor * old_rate) + (new_factor * new_value);
    
    response_rate_ = new_rate;

    return new_rate;
}