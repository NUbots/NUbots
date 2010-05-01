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
#include "debugverbositynusensors.h"
#include "../Kinematics/Horizon.h"
#include "Kinematics/Kinematics.h"
#include <math.h>
#include <boost/circular_buffer.hpp>
#include "Kinematics/Kinematics.h"
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
	m_kinematicModel = new Kinematics();
	m_kinematicModel->LoadModel("None");
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
	delete m_kinematicModel;
	m_kinematicModel = 0;
}

/*! @brief Updates and returns the fresh NUSensorsData. Call this function everytime there is new data.
    @return a pointer to the freshly updated sensor data
 */
NUSensorsData* NUSensors::update()
{
#if DEBUG_NUSENSORS_VERBOSITY > 0
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
#if DEBUG_NUSENSORS_VERBOSITY > 4
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
    
    calculateKinematics();
    calculateOrientation();
    calculateHorizon();
    calculateButtonTriggers();
    calculateFootForce();
    calculateFootImpact();
    calculateCoP();
    calculateZMP();
    calculateOdometry();
    calculateCameraHeight();
    calculateFallSense();
}

void NUSensors::calculateJointVelocity()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateJointVelocity()" << endl;
#endif
    static vector<float> previousjointpositions = m_data->JointPositions->Data;
    static vector<float> jointvelocities(m_data->JointPositions->Data.size(), 0);
    if (m_previous_time != 0)
    {
        for (unsigned i=0; i<m_data->JointPositions->Data.size(); i++)
        {
            jointvelocities[i] = 1000*(m_data->JointPositions->Data[i] - previousjointpositions[i])/(m_current_time - m_previous_time);
        }
    }
    m_data->JointVelocities->setData(m_current_time, jointvelocities, true);
    previousjointpositions = m_data->JointPositions->Data;
}

void NUSensors::calculateJointAcceleration()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateJointAcceleration()" << endl;
#endif
    static vector<float> previousjointvelocities = m_data->JointVelocities->Data;
    static vector<float> jointaccelerations(m_data->JointVelocities->Data.size(), 0);
    if (m_previous_time != 0)
    {
        for (unsigned i=0; i<m_data->JointVelocities->Data.size(); i++)
        {
            jointaccelerations[i] = 1000*(m_data->JointVelocities->Data[i] - previousjointvelocities[i])/(m_current_time - m_previous_time);
        }
    }
    m_data->JointAccelerations->setData(m_current_time, jointaccelerations, true);
    previousjointvelocities = m_data->JointVelocities->Data;
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
    m_data->BalanceHorizon->setData(m_current_time, line, true );
}


void NUSensors::calculateButtonTriggers()
{
        static float prevValueChest = 0.0f;
        static float prevValueLeftBumper = 0.0f;
        static float prevValueRightBumper = 0.0f;

        float pressTimeChest(0.0f);
        float pressTimeLeftBumper(0.0f);
        float pressTimeRightBumper(0.0f);

        if (m_previous_time != 0)
        {
            pressTimeChest = (*(m_data->ButtonTriggers))[0];
            pressTimeLeftBumper = (*(m_data->ButtonTriggers))[1];
            pressTimeRightBumper = (*(m_data->ButtonTriggers))[2];
        }

	vector<float> tempData;
        if(m_data->getButtonValues(NUSensorsData::MainButton, tempData) && (tempData.size() >= 1))
        {
            if(tempData[0] != prevValueChest)
                pressTimeChest = m_current_time;
            prevValueChest = tempData[0];
        }

        if(m_data->getFootBumperValues(NUSensorsData::AllFeet,tempData) && tempData.size() >= 2)
        {
            // Left Bumper
            if(tempData[0] != prevValueLeftBumper)
                pressTimeLeftBumper = m_current_time;
            prevValueLeftBumper = tempData[0];

            // Right Bumper
            if(tempData[1] != prevValueLeftBumper)
                pressTimeRightBumper = m_current_time;
            prevValueRightBumper = tempData[1];
        }

        // Now find the time since triggered and set to soft sensor value.
        tempData.clear();
        tempData.push_back(m_current_time - pressTimeChest);
        tempData.push_back(m_current_time - pressTimeLeftBumper);
        tempData.push_back(m_current_time - pressTimeRightBumper);
        m_data->ButtonTriggers->setData(m_current_time, tempData, true);
        return;
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
    //!< @todo Implement NUSensors::calculateCoP
}

void NUSensors::calculateOdometry()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateOdometry()" << endl;
#endif

    const float turnMultiplier = 1.0;
    const float xMultiplier = 1.25; // 2.5;
    const float yMultiplier = -1.0;

    static float prevHipYaw = 0.0;
    static float prevLeftX = 0.0;
    static float prevRightX = 0.0;
    static float prevLeftY = 0.0;
    static float prevRightY = 0.0;

    vector<float> leftFootPosition(3);
    vector<float> rightFootPosition(3);
    vector<float> odometeryData = m_data->Odometry->Data;
    if(odometeryData.size() < 3) odometeryData.resize(3,0.0); // Make sure the data vector is of the correct size.

    float hipYawPitch;
    m_data->getJointPosition(NUSensorsData::LHipYawPitch,hipYawPitch);

    const int arrayWidth = 4;
    const int translationCol = 3;

    // Get the left foot position relative to the origin
    vector<float> leftFootTransform = m_data->LeftLegTransform->Data;

    leftFootPosition[0] = leftFootTransform[translationCol];
    leftFootPosition[1] = leftFootTransform[translationCol+arrayWidth];
    leftFootPosition[2] = leftFootTransform[translationCol+2*arrayWidth];

    // Get the right foot position
    vector<float> rightFootTransform = m_data->RightLegTransform->Data;
    rightFootPosition[0] = rightFootTransform[translationCol];
    rightFootPosition[1] = rightFootTransform[translationCol+arrayWidth];
    rightFootPosition[2] = rightFootTransform[translationCol+2*arrayWidth];

    bool leftFootSupport = false;
    bool rightFootSupport = false;

    float leftForce, rightForce;
    m_data->getFootForce(NUSensorsData::LeftFoot,leftForce);
    m_data->getFootForce(NUSensorsData::RightFoot,rightForce);

    if(leftForce > rightForce)
    {
        leftFootSupport = true;
        rightFootSupport = false;
    }
    else
    {
        leftFootSupport = false;
        rightFootSupport = true;
    }
    
    // Calculate movement
    float deltaX;
    float deltaY;
    float deltaTheta;
    
    if(leftFootSupport && !rightFootSupport)
    {
        deltaX = xMultiplier * (leftFootPosition[0] - prevLeftX);
        deltaY = yMultiplier * (leftFootPosition[1] - prevLeftY);
        deltaTheta = turnMultiplier * (hipYawPitch - prevHipYaw);
    }
    else if(!leftFootSupport && rightFootSupport)
    {
        deltaX = xMultiplier * (rightFootPosition[0] - prevRightX);
        deltaY = yMultiplier * (rightFootPosition[1] - prevRightY);
        deltaTheta = turnMultiplier * (prevHipYaw - hipYawPitch);
    }
    
    odometeryData[0] += deltaX;
    odometeryData[1] += deltaY;
    odometeryData[2] += deltaTheta;

    m_data->Odometry->setData(m_data->CurrentTime, odometeryData, true);

    // Save the historical data
    prevHipYaw = hipYawPitch;
    prevLeftX = leftFootPosition[0];
    prevRightX = rightFootPosition[0];
    prevLeftY = leftFootPosition[1];
    prevRightY = rightFootPosition[1];
}

void NUSensors::calculateKinematics()
{
    //! TODO: Need to get these values from somewhere else.
    const bool leftLegSupport = true;
    const bool rightLegSupport = true;
    const int cameraNumber = 1;

    vector<float> leftLegJoints;
    bool leftLegJointsSuccess = m_data->getJointPositions(NUSensorsData::LeftLegJoints,leftLegJoints);
    vector<float> rightLegJoints;
    bool rightLegJointsSuccess = m_data->getJointPositions(NUSensorsData::RightLegJoints,rightLegJoints);
    vector<float> headJoints;
    bool headJointsSuccess = m_data->getJointPositions(NUSensorsData::HeadJoints,headJoints);

    Matrix rightLegTransform;
    Matrix leftLegTransform;
    Matrix bottomCameraTransform;
    Matrix* supportLegTransform = 0;
    Matrix* cameraTransform = 0;

    // Calculate the transforms
    if(rightLegJointsSuccess)
    {
        rightLegTransform = m_kinematicModel->CalculateTransform(Kinematics::rightFoot,rightLegJoints);
    }
    if(leftLegJointsSuccess)
    {
        leftLegTransform = m_kinematicModel->CalculateTransform(Kinematics::leftFoot,leftLegJoints);
    }
    if(headJointsSuccess)
    {
        bottomCameraTransform = m_kinematicModel->CalculateTransform(Kinematics::bottomCamera,headJoints);        
    }

    // Select the appropriate ones for further calculations.
    // Choose camera.
    if(headJointsSuccess && (cameraNumber == 1))
    {
        cameraTransform = &bottomCameraTransform;
    }

    // Choose support leg.
    if((!leftLegSupport && rightLegSupport) && rightLegJointsSuccess)
    {
        supportLegTransform = &rightLegTransform;
    }
    else if((leftLegSupport && !rightLegSupport) && leftLegJointsSuccess)
    {
        supportLegTransform = &leftLegTransform;
    }
    else if((leftLegSupport && rightLegSupport) && leftLegJointsSuccess && rightLegJointsSuccess)
    {
        supportLegTransform = &leftLegTransform;
    }
    else
    {
        supportLegTransform = 0;
    }

    // Set all of the sensor values
    double time = m_data->CurrentTime;
    // Set the legs
    m_data->LeftLegTransform->setData(time,leftLegTransform.asVector(),true);
    m_data->RightLegTransform->setData(time,rightLegTransform.asVector(),true);

    // Set the camera
    m_data->CameraTransform->setData(time, cameraTransform->asVector(), true);

    if(supportLegTransform)
    {
        m_data->SupportLegTransform->setData(time,supportLegTransform->asVector(),true);

        // Calculate transfrom matrix to convert camera centred coordinates to ground centred coordinates.
        Matrix cameraToGroundTransform = Kinematics::CalculateCamera2GroundTransform(*supportLegTransform, *cameraTransform);
        m_data->CameraToGroundTransform->setData(time, cameraToGroundTransform.asVector(), true);
    }
    else
    {
        m_data->SupportLegTransform->IsValid = false;
        m_data->CameraToGroundTransform->IsValid = false;
    }
    return;
}

void NUSensors::calculateCameraHeight()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateCameraHeight()" << endl;
#endif
    const int arrayLocation = 3 + 2*4; // collumn index is 3, number of collumns per row is 4.
    float cameraHeight = m_data->CameraToGroundTransform->Data[arrayLocation];
    m_data->CameraHeight->setData(m_data->CurrentTime, vector<float>(1, cameraHeight));
}



