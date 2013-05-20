#include <algorithm>
#include <iomanip>
#include "CM730.h"
#include "SensorReadDescriptor.h"
using namespace Robot;

// NOTE: TEST: DEBUG: REFACTOR:
// These defines everywhere should obviously be replaced by an enum ASAP.
#define SENSOR_ERROR_NO_RESPONSE        (-1)

// Update a response rate estimate given an error value.
// It doesn't matter how this is done, so long as it's relatively fast
// and results in reasonable performance.
double SensorReadDescriptor::UpdateResponseRate(int sensor_error_code)
{
    double old_rate = response_rate_;
    // double new_value = (error_code == 0)? 1.0 : 0.0;
    double new_value = (sensor_error_code != SENSOR_ERROR_NO_RESPONSE)? 1.0 : 0.0;

    ++consecutive_errors_;
    if(new_value) consecutive_errors_ = 0;
    
    // This value is arbitrary, and should be tuned for reasonable performance
    double old_factor = 0.75;
    double new_factor = 1 - old_factor;
    double new_rate = (old_factor * old_rate) + (new_factor * new_value);
    
    // Clip the new response rate:
    // (letting it fall too low will mean it will take a long time to recover)
    new_rate = std::min(1.0, std::max(0.00001, new_rate));

    response_rate_ = new_rate;
    return new_rate;
}