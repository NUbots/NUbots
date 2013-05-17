#include <cmath>
#include "SensorReadDescriptor.h"
#include "CompareSensorReadDescriptors.h"
using namespace Robot;

// Compare two SensorReadDescriptor pointers.
bool CompareSensorReadDescriptors::operator()(
    SensorReadDescriptor*& sensor_read_a, 
    SensorReadDescriptor*& sensor_read_b) const
{
    // // Sort first by response rate...
    // double rr_a = sensor_read_a->response_rate();
    // double rr_b = sensor_read_b->response_rate();
    // double diff = std::abs(rr_a - rr_b);
    // if(diff > 0.0001)
    //     return rr_a > rr_b; // highest first

    // then by consecutive errors...
    double errors_a = sensor_read_a->consecutive_errors();
    double errors_b = sensor_read_b->consecutive_errors();
    if(errors_a != errors_b)
        return errors_a < errors_b; // lowest first

    // then by sensor id.
    double id_a = sensor_read_a->sensor_id();
    double id_b = sensor_read_b->sensor_id();
    return id_a < id_b; // lowest first
}
