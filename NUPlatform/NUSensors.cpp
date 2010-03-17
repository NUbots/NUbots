/*! @file NUSensors.cpp
    @brief Partial implementation of base sensor class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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
#include "debug.h"
#include "../Kinematics/Horizon.h"
#include <math.h>
#include <boost/circular_buffer.hpp>

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
    m_previous_time = 0;
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
    if (m_data != NULL)
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
    calculateSoftSensors();
    
#if DEBUG_NUSENSORS_VERBOSITY > 3
    static bool firstrun = true;
    if (firstrun)
    {
        debug << "NUSensors::update(). Available Sensors:" << endl;
        m_data->summaryTo(debug);
        firstrun = false;
    }
#endif
#if DEBUG_NUSENSORS_VERBOSITY > 5
    debug << "NAOWebotsSensors::NAOWebotsSensors():" << endl;
    m_data->summaryTo(debug);
#endif
    m_previous_time = m_current_time;
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
    calculateHorizon();
    calculateFootForce();
    calculateFootImpact();
    calculateCoP();
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
        float accelsum = sqrt(pow(acceleration[0],2) + pow(acceleration[1],2) + pow(acceleration[2],2));
        if (fabs(accelsum - 981) < 0.1*981)
        {   // only update the orientation estimate if not under other accelerations!
            orientation[0] = atan2(-acceleration[1],-acceleration[2]);
            orientation[1] = atan2(acceleration[0],-acceleration[2]);
            orientation[2] = atan2(acceleration[1],acceleration[0]);            // this calculation is pretty non-sensical
        }
        m_data->BalanceOrientation->setData(m_current_time, orientation, true);
    }
}

/*! @brief Updates the Horizon Line using the current sensor data
    
 */
void NUSensors::calculateHorizon()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateHorizon()" << endl;
#endif
    Horizon HorizonLine;
    float bodyPitch;
    float bodyRoll;
    bodyRoll = m_data->BalanceOrientation->Data[0];
    bodyPitch = m_data->BalanceOrientation->Data[1];
    float headYaw;
    m_data->getJointPosition(NUSensorsData::HeadYaw,headYaw);
    float headPitch;
    m_data->getJointPosition(NUSensorsData::HeadPitch,headPitch);
    int camera = 1;

    HorizonLine.Calculate((double)bodyPitch,(double)bodyRoll,(double)-headYaw,(double)headPitch,camera);
    vector<float> line;
    line.push_back(HorizonLine.getA());
    line.push_back(HorizonLine.getB());
    line.push_back(HorizonLine.getC());
    //debug << "HLINE IS: "<<line[0] << line[1] << line[2] << endl;
    m_data->BalanceHorizon->setData(m_current_time, line, true );
/*    static vector<float> orientation(3, 0);
    static vector<float> acceleration(3, 0);
    if (m_data->getAccelerometerValues(acceleration))
    {
        float accelsum = sqrt(pow(acceleration[0],2) + pow(acceleration[1],2) + pow(acceleration[2],2));
        if (fabs(accelsum - 981) < 0.1*981)
        {   // only update the orientation estimate if not under other accelerations!
            orientation[0] = atan2(-acceleration[1],-acceleration[2]);
            orientation[1] = atan2(acceleration[0],-acceleration[2]);
            orientation[2] = atan2(acceleration[1],acceleration[0]);            // this calculation is pretty non-sensical
        }
        m_data->BalanceOrientation->setData(m_current_time, orientation, true);
    }
*/

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
    This function should be called after calculateOrientation and calculateZMP so that those results are available.
 
    Except in special circumstances, a fall is always preceded by a falling. We transition into the falling state
    when no control action can prevent a fall. The falling is completed after the robot has impacted with the ground, 
    the impact is detected using the accelerometers, and once the robot has settled it is fallen and can start getting up.
 
    To handle the special cases where the robot is fallen without ever having fell, ie. on Initial, on Ready and on Playing
    ......................................
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
    // Can I ever be fallen, without falling?
    // On initial. On ready. On playing. These are special cases where the fall happened outside the game.
    // I could use the game state to trigger a special check, or I could just check if I have fallen over
    // in every frame.
    // Alas, in init we are not going to have any motors on. When we go to ready, I need to stand up.
    // So I could check whether I am standing in each frame, to do this I would need to know the stance
    // position, if the feet are on the ground, and if the orientation is upright. That is a probably because, 
    // I don't have the stance position in the sensors.
    //! @todo TODO: Finish this discussion on how to do the fall detection...
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

/*! @brief Calculates the total force in Newtons on each foot
 
    The total force on each foot is calibrated online assuming that over time the force on the left and right foot
    will average out to be the same value.
 */
void NUSensors::calculateFootForce()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateFootForce()" << endl;
#endif
    static vector<float> forces(3,0);
    const float MINIMUM_CONTACT_FORCE = 2;          // If we register a force less than 2N (approx. 200g)
    forces[0] = 0;
    // now I assume that the left foot values are stored first (because they are ;)) and that the left and right feet have the same number of sensors
    for (int i=0; i<m_data->FootSoleValues->size()/2; i++)
        forces[0] += (*m_data->FootSoleValues)[i];
    forces[1] = 0;
    for (int i=m_data->FootSoleValues->size()/2; i<m_data->FootSoleValues->size(); i++)
        forces[1] += (*m_data->FootSoleValues)[i];
    
    // Now I attempt to compensate for uncalibrated foot sensors by using the long term averages, to scale each foot
    // I assume that the force on the left and right foot over time should be the same.
    static float leftforcesum = 0;
    static float rightforcesum = 0;
    static int forcecount = 0;
    static float leftforceaverage = 0;
    static float rightforceaverage = 0;
    
    if (forces[0] + forces[1] > MINIMUM_CONTACT_FORCE)
    {   // only use forces which exceed a minimum contact force
        leftforcesum += forces[0];
        rightforcesum += forces[1];
        forcecount++;
        leftforceaverage = leftforcesum/forcecount;
        rightforceaverage = rightforcesum/forcecount;
        
        if (forcecount > 500)
        {   // only scale using the long term averages, if we have enough data
            forces[0] *= (leftforceaverage + rightforceaverage)/leftforceaverage;
            forces[1] *= (leftforceaverage + rightforceaverage)/rightforceaverage;
        }
        
        if (forcecount > 5000)
        {   // if the number of forcecount is getting large, then the sums are getting large, and we should avoid overflow
            forcecount = 2000;
            leftforcesum = leftforceaverage*forcecount;
            rightforcesum = rightforceaverage*forcecount;
        }
    }
    
    // save the foot forces in the FootForce sensor
    forces[2] = forces[0] + forces[1];
    m_data->FootForce->setData(m_current_time, forces, true);
}

/*! @brief Determines the time at which each foot last impacted with the ground

 */
void NUSensors::calculateFootImpact()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateFootImpact()" << endl;
#endif   
    const float MINIMUM_CONTACT_FORCE = 2;          // If we register a force less than 2N (approx. 200g)
    static vector<float> impacttimes(2,0);
    static vector<float> previousimpacttimes(2,0);
    
    // The idea is to keep track of what is 'small' and what is 'large' for foot forces
    // And then define an impact to be a transisition from small to large foot force
    if (m_previous_time != 0)
    {
        // grab the current force readings
        float leftforce = (*(m_data->FootForce))[0];
        float rightforce = (*(m_data->FootForce))[1];
        float totalforce = (*(m_data->FootForce))[2];
        
        const int numpastvalues = static_cast<int> (200.0/(m_current_time - m_previous_time));
        static boost::circular_buffer<float> previousleftforces(numpastvalues, 0);      // forces on the left foot
        static boost::circular_buffer<float> previousrightforces(numpastvalues, 0);     // forces on the right foot
        static boost::circular_buffer<float> previoustotalforces(numpastvalues, 0);     // forces on the right foot
        static float leftforcemin = leftforce;
        static float leftforcemax = leftforce;
        static float rightforcemin = rightforce;
        static float rightforcemax = rightforce;
        static unsigned int forcecount = 0;
        
        previousleftforces.push_back(leftforce);
        previousrightforces.push_back(rightforce);
        previoustotalforces.push_back(totalforce);
        
        // calculate the average left and right forces
        float leftsum = 0;
        float rightsum = 0;
        for (unsigned int i=0; i<previousleftforces.size(); i++)
        {
            leftsum += previousleftforces[i];
            rightsum += previousrightforces[i];
        }
        float leftavg = leftsum/numpastvalues;
        float rightavg = rightsum/numpastvalues;
        
        // update the left and right min and max values
        if (leftavg < leftforcemin)
            leftforcemin = leftavg;
        else if (leftavg > leftforcemax)
            leftforcemax = leftavg;
        else
        {   // we slowly decay the min and max values to implement a form of min/max tracking
            leftforcemin *= 1.0005;
            leftforcemax *= 0.9995;
        }
        
        if (rightavg < rightforcemin)
            rightforcemin = rightavg;
        else if (rightavg > rightforcemax)
            rightforcemax = rightavg;
        else 
        {
            rightforcemin *= 1.0005;
            rightforcemax *= 0.9995;
        }
        
        // now I know what 'small' is, I define an impact to be a transition from 'small' to not 'small'
        if (leftforcemax > 3*leftforcemin)
        {   // we can only use the foot sensors to detect impacts if there is sufficent range
            bool leftprevioussmall = true;
            for (unsigned int i=0; i<previousleftforces.size()-1; i++)
            {
                if (previoustotalforces[i] < MINIMUM_CONTACT_FORCE)
                    leftprevioussmall = false;
                else if (previousleftforces[i] > (leftforcemin + leftforcemax)/4.0)
                    leftprevioussmall = false;
            }
            if (leftprevioussmall == true && leftforce > (leftforcemin + leftforcemax)/4.0)
            {
                previousimpacttimes[0] = impacttimes[0];
                impacttimes[0] = m_current_time;
                for (unsigned int i=0; i<previousleftforces.size(); i++)
                {
                    debug << " " << previousleftforces[i];
                }
                debug << endl;
            }
        }
        if (rightforcemax > 3*rightforcemin)
        {   // again only use the foot sensors if there is sufficient range
            bool rightprevioussmall = true;
            for (unsigned int i=0; i<previousrightforces.size()-1; i++)
            {
                if (previoustotalforces[i] < MINIMUM_CONTACT_FORCE)
                    rightprevioussmall = false;
                else if (previousrightforces[i] > (rightforcemin + rightforcemax)/4.0)
                    rightprevioussmall = false;
            }
            if (rightprevioussmall == true && rightforce > (rightforcemin + rightforcemax)/4.0)
            {
                previousimpacttimes[1] = impacttimes[1];
                impacttimes[1] = m_current_time;
                for (unsigned int i=0; i<previousrightforces.size(); i++)
                {
                    debug << " " << previousrightforces[i];
                }
                debug << endl;
            }
        }
    }
    
    m_data->FootImpact->setData(m_data->CurrentTime, impacttimes, true);
}

void NUSensors::calculateCoP()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateCoP()" << endl;
#endif
}

