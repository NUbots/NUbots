
#ifndef COMPARE_SENSOR_READ_DESCRIPTORS_H
#define COMPARE_SENSOR_READ_DESCRIPTORS_H

#include "SensorReadDescriptor.h"

namespace Robot
{
    class CompareSensorReadDescriptors
    {
    public:
        // Compare two SensorReadDescriptor pointers.
        bool operator()(SensorReadDescriptor*& sensor_read_a,
                        SensorReadDescriptor*& sensor_read_b) const;
    };
}

#endif
