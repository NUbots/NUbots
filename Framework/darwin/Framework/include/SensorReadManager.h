#ifndef SENSOR_READ_MANAGER_H
#define SENSOR_READ_MANAGER_H

#include <vector>
#include <boost/unordered_map.hpp>

#include "CM730.h"
#include "SensorReadManager.h"
#include "SensorReadDescriptor.h"

namespace Robot
{
    class BulkReadData;
}

namespace Robot
{
    //! A container for read descriptors.
    //! Supports constant-time lookup by sensor_id, and iteration in order of
    //! decreasing response_rate.
    class SensorReadManager
    {
    public:
        //! Creates data structures and sensor descriptors.
        SensorReadManager();

        //! Destructor
        ~SensorReadManager();

        //! Allows the sensor read descriptors to be accessed by sensor id
        // SensorReadDescriptor& operator[](const int sensor_id);
        SensorReadDescriptor* GetDescriptorById(int sensor_id);

        //! Processes all error information returned from a bulk read,
        //! and updates sensor response rates accordingly.
        bool ProcessBulkReadErrors(
            int bulk_read_error_code, 
            BulkReadData* bulk_read_data);

        //! Copies descriptor data to the given transmit packet buffer,
        //! ordering the sensors such that the number that are expected to
        //! respond is high.
        void MakeBulkReadPacket(unsigned char* tx_packet);

        //! Checks all sensors for bulk read errors and returns true if any
        //! 'significant' errors occured.
        bool CheckSensorsBulkReadErrors(BulkReadData* bulk_read_data);

        //! Gets the bulk read errors from the given sensor
        int GetSensorBulkReadErrors(
            SensorReadDescriptor* sensor_read,
            BulkReadData* bulk_read_data);

        //! Checks the severity of the given error for the given sensor
        //! and takes an appropriate action.
        bool CheckSensorBulkReadErrors(
            SensorReadDescriptor* sensor_read,
            int sensor_error_code);

        //! Updates all sensor response rates using the same error code for each
        void UpdateSensorResponseRates(int error_code);

        //! Pretty prints the response rate of all sensors
        void PrintSensorResponseRates();

        //! Pretty prints the response rate of a single sensors
        void PrintSensorResponseRate(int sensor_id);

        //! Filters the list of response rates to return a list of sensor IDs that
        //! are actually failing.
        //! (i.e. Removes likely false-positives from the list of failing sensors.
        //! Currently assumes that all sensors are always in use.
        //! Note: If three sensors on the same limb are not responding, it is likely 
        //!       that only the first of them requires attention)
        void GetFilteredLikelySensorFailures(std::vector<int>* failing_sensors);

        //! When the CM730 board is reset, all it's sensors stop responding,
        //! except for the CM.
        //! This method returns true if this is the case.
        bool CheckForCM730ResetState();

        //! Returns a string containing a list of descriptions of the set error
        //! flags in the given errorvalue.
        std::string GetSensorErrorDescription(unsigned int error_value);

        //! Returns the name of the sensor with the given id as a string
        static const char* SensorNameForId(int sensor_id);

    private:
        void FilterLimbSensorFailures(
            std::vector<int>& limb_sensors, 
            std::vector<int>& failing_sensors);

        // Note: The following data structures should be moved into a
        //       new SensorDescriptorContainer class that
        //       implements the required iterators and access operations.

        //! A vector to store the actual descriptors
        std::vector<SensorReadDescriptor*> descriptor_list_;

        // Data structures containing pointers to elements of descriptor_list.
        // (Used to support SensorReadManager's access requirements) 
        boost::unordered_map<int, SensorReadDescriptor*> descriptor_map_;

        // Note: A heap may not be the fastest option.
        // Use 'SensorReadManager::GetDescriptorById(int sensor_id)'
        // instead of acessing the map directly.
        std::vector<SensorReadDescriptor*> descriptor_heap_;
    };
}

#endif
