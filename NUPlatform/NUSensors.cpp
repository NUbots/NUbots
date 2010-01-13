/*! @file NUSensors.cpp
    @brief Partial implementation of base sensor class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "NUSensors.h"
#include "NUSystem.h"
#include "Tools/debug.h"

#include <math.h>
using namespace std;

/*! @brief Default constructor for parent NUSensors class, this will/should be called by children
 
    Creates the NUSensorsData instance (m_data) in which all of the sensor data is stored.
 */
NUSensors::NUSensors()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::NUSensors" << endl;
#endif
    m_current_time = nusystem->getTime();
    m_data = new NUSensorsData();
}

/*! @brief Destructor for parent NUSensors class.
 
    Deletes the NUSensorsData instance (m_data)
 */
NUSensors::~NUSensors()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::~NUSensors" << endl;
#endif
    delete m_data;
}

/*! @brief Updates and returns the fresh NUSensorsData. Call this function everytime there is new data.
    @return a pointer to the freshly updated sensor data
 */
NUSensorsData* NUSensors::update()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::update()" << endl;
#endif
    m_current_time = nusystem->getTime();
    copyFromHardwareCommunications();       // the implementation of this function will be platform specific
    return getData();
}

/*! @brief Returns a pointer to the current NUSensorsData.
 
    @attention I do not copy any of the NUSensorsData, and the current data could be updated at any time.
               Consequently, if you want to save some data to be synchronised with vision you have to make
               the copy of what you need yourself. However, the getJointPositions etc functions of NUSensorsData
               will return copies of the selected data.
 */
NUSensorsData* NUSensors::getData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::getData()" << endl;
#endif
    return m_data;
}

/*! @brief The lower level function which gets the sensor data from the hardware itself. 
           This is a dummy function, and must be implemented by all children.
 */
void NUSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::copyFromHardwareCommunications()" << endl;
#endif
    // Implementation in Platforms!
}

void NUSensors::calculateSoftSensors()
{
    // if there is no velocity sensor calculate velocity
    if (m_data->JointVelocities->IsValid == false || m_data->JointVelocities->IsCalculated == true)
        calculateJointVelocity();
    // if there is no acceleration sensor
    if (m_data->JointAccelerations->IsValid == false || m_data->JointAccelerations->IsCalculated == true)
        calculateJointAcceleration();
    
    calculateOrientation();
    calculateZMP();
    calculateFallSense();
}

void NUSensors::calculateJointVelocity()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateJointVelocity()" << endl;
#endif
}

void NUSensors::calculateJointAcceleration()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateJointAcceleration()" << endl;
#endif
}

/*! @brief Updates the orientation estimate using the current sensor data
    @todo TODO: Implement this function properly with EKF etc. This will also update gyro and accel readings
 */
void NUSensors::calculateOrientation()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateOrientation()" << endl;
#endif
    static vector<float> orientation(3, 0);
    static vector<float> acceleration(3, 0);
    if (m_data->getAccelerometerValues(acceleration))
    {
        orientation[0] = atan2(-acceleration[1],-acceleration[2]);
        orientation[1] = atan2(acceleration[0],-acceleration[2]);
        orientation[2] = atan2(acceleration[1],acceleration[0]);            // this calculation is pretty non-sensical
        m_data->BalanceOrientation->setData(m_current_time, orientation, true);
    }
}

/*! @brief Updates the zero moment point estimate using the current sensor data
    @todo TODO: Implement this function. Fuse data from inverted pendulum ZMP, CoP and kinematic ZMP
 */
void NUSensors::calculateZMP()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateZMP()" << endl;
#endif
}

/*! @brief Updates the fall sense (ie BalanceFalling and BalanceFallen) using the current sensor data.
 
    Both BalanceFalling and BalanceFallen are [sum, left, right, forward, backward] where 0=not fall(ing) 1=fall(ing).
    This function should be called after calculateOrientation and calculateZMP so that those results are available to it
 */
void NUSensors::calculateFallSense()
{
    static const float Fallen = 1.0;
    static const float RollFallenThreshold = 1.1;       // approx. 60 deg. The falling threshold will be approx 30 deg
    static const float PitchFallenThreshold = 1.22;     // approx. 70 deg.
    static const float Falling = 1.0;
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateFallingSense()" << endl;
#endif
    // check if the robot has fallen over
    static vector<float> orientation(3,0);
    static vector<float> angularvelocity(3,0);
    static vector<float> falling(5,0);
    static vector<float> fallen(5,0);
    
    m_data->getOrientation(orientation);
    m_data->getGyroValues(angularvelocity);
    
    // check if fallen left
    if (orientation[0] < -RollFallenThreshold)
        fallen[1] = Fallen;
    else
        fallen[1] = 0.0;
    // check if fallen right
    if (orientation[0] > RollFallenThreshold)
        fallen[2] = Fallen;
    else
        fallen[2] = 0.0;
    // check if fallen forward
    if (orientation[1] > RollFallenThreshold)
        fallen[3] = Fallen;
    else
        fallen[3] = 0.0;
    // check if fallen backward
    if (orientation[1] < -RollFallenThreshold)
        fallen[4] = Fallen;
    else
        fallen[4] = 0.0;
    fallen[0] = fallen[1] + fallen[2] + fallen[3] + fallen[4];
    m_data->BalanceFallen->setData(m_current_time, fallen, true);
    // check if the robot is falling over
    falling[0] = falling[1] + falling[2] + falling[3] + falling[4];
    m_data->BalanceFalling->setData(m_current_time, falling, true);
}



