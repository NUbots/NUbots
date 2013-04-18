/*! @file DarwinSensors.h
    @brief Declaration of Darwin sensors class
 
    @class DarwinSensors
    @brief A Darwin sensors
 
    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DARWINSENSORS_H
#define DARWINSENSORS_H

#include <vector>
#include <boost/unordered_map.hpp>
#include "NUPlatform/NUSensors.h"
#include "Infrastructure/NUData.h"


class DarwinJointMapping;
class DarwinPlatform;
namespace Robot
{
    class CM730;
}

class DarwinSensors : public NUSensors
{
public:
    DarwinSensors(DarwinPlatform*, Robot::CM730*);
    ~DarwinSensors();
    
    void copyFromHardwareCommunications();

    void copyFromJoints();
    void copyFromAccelerometerAndGyro();
    void copyFromFeet();
    void copyFromButtons();
    void copyFromBattery();
    
protected:
    //! A vector containing pointers to all of the joint id_t.
    //! This is used to loop through all of the joints quickly
    vector<NUData::id_t*> m_joint_ids;
    vector<float> m_previous_positions;
    vector<float> m_previous_velocities;
    DarwinPlatform* platform;
    Robot::CM730* cm730;
    DarwinJointMapping* m_joint_mapping;

    // Current percentage of reads from this sensor that do not result in an error
    boost::unordered_map<int, double> sensor_response_rates;

    //! A flag to indicate a motor indicated an error
    bool motor_error;
    /// Returns a string containing a list of descriptions of the set error
    /// flags in the given errorvalue.
    std::string getSensorErrorDescription(unsigned int error_value);
    //! Prints bulk read errors for all servos and returns true if any occured.
    bool CheckServosBulkReadErrors();
    //! Checks a single sensor/servo for bulk read errors, prints them, and
    //! returns whether or not any occured.
    bool CheckSensorBulkReadErrors(int sensor_id);
    
    //! Initialises the mapping of sensor values to their respective response
    //! rates.
    //! Should be called before the first call to UpdateSensorResponseRate(...).
    void InitialiseSensorResponseRates();
    
    //! Updates the response rate estimate for the given sensor using the
    //! given sensor error code.
    //! returns the updated response rate.
    double UpdateSensorResponseRate(int sensor_id, int error_code);

    //! Updates all sensor response rates using the same error code for each
    double UpdateSensorResponseRates(int error_code);


    //! Pretty prints the response rate of all sensors
    void PrintSensorResponseRates();
    //! Pretty prints the response rate of a single sensors
    void PrintSensorResponseRate(int sensor_id);


    // Feel free to make this method public if necessary. -MM
    static const char* GetSensorName(int joint_id);

private:
    static const unsigned int NUM_MOTORS = 20;
};

#endif

