#ifndef SENSOR_READ_MANAGER_H
#define SENSOR_READ_MANAGER_H

#include "CM730.h"
#include "SensorReadManager.h"
#include "SensorReadDescriptor.h"

namespace Robot
{
    //! A container for read descriptors.
    //! Supports constant-time lookup by sensor_id, and iteration in order of
    //! decreasing response_rate.
    class SensorReadManager
    {
    public:
        void Initialize();
        void MakeBulkReadPacket(unsigned char* tx_packet);
        void GetFilteredLikelySensorFailures(std::vector<int>* failing_sensors);
        SensorReadDescriptor& operator[](const int index);
        // void InitialiseSensorResponseRates();
        void UpdateSensorResponseRates(int error_code);
        double UpdateSensorResponseRate(int sensor_id, int error_code);
        void PrintSensorResponseRates();
        void PrintSensorResponseRate(int sensor_id);

        //! Returns the name of the sensor with the given id as a string
        static const char* SensorNameForId(int sensor_id);

    private:
        void FilterLimbSensorFailures(
            std::vector<int>& limb_sensors, 
            std::vector<int>& failing_sensors);

        //! A vector to store the actual descriptors
        std::vector<SensorReadDescriptor> descriptor_list;
        // Data structures containing pointers to elements of descriptor_list.
        // (Used to support SensorReadManager's access requirements) 
        boost::unordered_map<int, SensorReadDescriptor*> descriptor_map;
        // Note: A heap is probably an awful choice for this
        std::vector<SensorReadDescriptor*> descriptor_heap;
    };
}

#endif
