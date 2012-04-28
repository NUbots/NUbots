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
#include <cassert>

#include "NUSensors.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "NUPlatform/NUPlatform.h"

#include "NUSensors/EndEffectorTouch.h"
#include "NUSensors/OdometryEstimator.h"
#include "Kinematics/Horizon.h"
#include "Kinematics/Kinematics.h"
#include "Kinematics/OrientationUKF.h"

#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"
#include "Motion/Tools/MotionFileTools.h"
#include "Tools/Math/FIRFilter.h"

#include "debug.h"
#include "debugverbositynusensors.h"
#include "nubotdataconfig.h"

#include <math.h>
#include <limits>
using namespace std;

/*! @brief Default constructor for parent NUSensors class, this will/should be called by children
 
    Creates the NUSensorsData instance (m_data) in which all of the sensor data is stored.
 */
NUSensors::NUSensors(): m_initialised(false)
{
#if DEBUG_NUSENSORS_VERBOSITY > 0
    debug << "NUSensors::NUSensors" << endl;
#endif
    m_current_time = Platform->getTime();
    m_previous_time = -1000;
    m_data = new NUSensorsData();
    m_touch = new EndEffectorTouch();
    m_kinematicModel = new Kinematics();
    m_kinematicModel->LoadModel();
    m_orientationFilter = new OrientationUKF();
    m_odometry = new OdometryEstimator();
}

/*! @brief Destructor for parent NUSensors class.
 
    Deletes the NUSensorsData instance (m_data)
 */
NUSensors::~NUSensors()
{
#if DEBUG_NUSENSORS_VERBOSITY > 0
    debug << "NUSensors::~NUSensors" << endl;
#endif
    delete m_kinematicModel;
    m_kinematicModel = 0;
    delete m_orientationFilter;
    m_orientationFilter = 0;
    delete m_odometry;
    m_odometry = 0;
}

/*! @brief Updates and returns the fresh NUSensorsData. Call this function everytime there is new data.
    @return a pointer to the freshly updated sensor data
 */
void NUSensors::update()
{
#if DEBUG_NUSENSORS_VERBOSITY > 0
    debug << "NUSensors::update()" << endl;
#endif
    m_data->PreviousTime = m_data->CurrentTime;
    m_current_time = Platform->getTime();
    m_data->CurrentTime = m_current_time;
    copyFromHardwareCommunications();       // the implementation of this function will be platform specific
    calculateSoftSensors();
    
#if DEBUG_NUSENSORS_VERBOSITY > 0
    m_data->summaryTo(debug);
#endif
    m_previous_time = m_current_time;
}

/*! @brief Returns a pointer to the NUSensorsData object used to store sensor values */
NUSensorsData* NUSensors::getNUSensorsData()
{
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
    if(!m_initialised) initialise();
    m_touch->calculate();
    calculateKinematics();
    calculateOrientation();
    calculateHorizon();
    calculateButtonDurations();

    //m_zmp->calculate();

    calculateOdometry();
    calculateCameraHeight();
    calculateFallSense();
}

/*! @brief Intialisation function to fill in the tables required for the new generalised kinematic model.
           After running this function the variables m_kinematics_joint_map, m_kinematics_transform_ids, and m_kinematics_effector_ids
           will have been updated with the values obtained from the Kinematics model and the NUSensorsData available.
           Because the complete NUSensorsData is not fully available during the constructor of this class this action must be dealayed until the first update.
 */
void NUSensors::initialise()
{
    // Need to initialise a bunch of tables to prevent doing string compares with the kinematics model.
    m_initialised = true;   // Flag that this function has been run.

    // Get the kinematic model data
    const Kinematics::RobotModel* model = m_kinematicModel->getModel();
    std::vector<const NUData::id_t*> effector_buffer;

    // Clear all of the tables.
    m_kinematics_map.clear();

    unsigned int index = 0; // variable to keep track of the vector indec we are working with.
    // Cycle through all of the end effectors of the kinematic model.
    for(Kinematics::RobotModel::const_iterator effector_it = model->begin(); effector_it != model->end(); ++effector_it)
    {
        effector_buffer.clear();    // Clear out old info.
        // Now go through each of the links in the end effector.
        for(std::vector<Link>::const_iterator link_it = effector_it->links()->begin(); link_it != effector_it->links()->end(); ++link_it)
        {
            // Get name of the link.
            std::string link_name = link_it->name();
            // Now find the id required to retrieve the data for this joint.
            NUData::id_t* p_id = m_data->getId(link_name);
            assert(p_id);   // make sure we got an id.
            effector_buffer.push_back(p_id);    // add to the list for this effector.
        }
        // Add the list of joint id's to the list.
        std::string effector_name = effector_it->name();
        const NUData::id_t* p_effector_id = NULL;
        const NUData::id_t* p_transform_id = NULL;

        // Check if the effector is one that is stored in the sensor data.
        if(effector_name == "Bottom Camera" or effector_name == "Camera")   // On NAO we just use the bottom camera.
        {
            if(effector_name == "Bottom Camera") m_camera_number = 1;
            else m_camera_number = 0;
            p_transform_id = &NUSensorsData::CameraTransform;
            p_effector_id = NULL;       // No end effector for the camera.
        }
        else if(effector_name == "Left Foot")
        {
            p_transform_id = &NUSensorsData::LLegTransform;
            p_effector_id = &NUSensorsData::LLegEndEffector;
        }
        else if(effector_name == "Right Foot")
        {
            p_transform_id = &NUSensorsData::RLegTransform;
            p_effector_id = &NUSensorsData::RLegEndEffector;
        }
        else
        {
            ++index;
            continue;   // don't care about updating effectors we do not use the values for.
        }

        // Add all of the values that we have discovered to the table.
        KinematicMap temp;
        temp.joints = effector_buffer;
        temp.effector_id = p_effector_id;
        temp.transform_id = p_transform_id;
        temp.index = index;
        m_kinematics_map.push_back(temp);
        ++index;
    }

#if DEBUG_NUSENSORS_VERBOSITY > 0
    debug << "Kinematic table initialisation:" <<std::endl << std::endl;

    for (std::vector<KinematicMap>::iterator f_it = m_kinematics_map.begin(); f_it != m_kinematics_map.end(); ++f_it)
    {
        debug << "Index: " << f_it->index << " Transfrom ID: " << f_it->transform_id;
        if(f_it->effector_id) debug << " effector_id: " << f_it->effector_id;
        debug << std::endl;
        debug << "Joints: ";
        for(std::vector<const NUData::id_t*>::iterator j_it = f_it->joints.begin(); j_it != f_it->joints.end(); ++j_it)
        {
            debug << (*j_it) << " ";
        }
        debug << std::endl << std::endl;
    }
#endif

    return;
}

/*! @brief Updates the orientation estimate using the current sensor data
 */
void NUSensors::calculateOrientation()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateOrientation()" << endl;
#endif
    static vector<float> orientation(3, 0.0f);
    static vector<float> orientationhardware(3, 0.0f);
    static vector<float> acceleration(3, 0.0f);
    static vector<float> gyros(3, 0.0f);
    static vector<float> gyroOffset(3, 0.0f);
    
    if (m_data->get(NUSensorsData::OrientationHardware, orientationhardware))
    {
        m_data->set(NUSensorsData::Orientation, m_current_time, orientationhardware);
    }
    else if (m_data->get(NUSensorsData::Gyro, gyros) && m_data->get(NUSensorsData::Accelerometer, acceleration))
    {
        vector<float> supportLegTransformFlat;
        bool validKinematics = m_data->get(NUSensorsData::SupportLegTransform, supportLegTransformFlat);
        Matrix supportLegTransform = Matrix4x4fromVector(supportLegTransformFlat);
        if(validKinematics)
            orientation = Kinematics::OrientationFromTransform(supportLegTransform);

        validKinematics = validKinematics && (fabs(acceleration[2]) > 2*fabs(acceleration[1]) && fabs(acceleration[2]) > 2*fabs(acceleration[0]));

        if(!m_orientationFilter->Initialised())
            m_orientationFilter->initialise(m_current_time,gyros,acceleration,validKinematics,orientation);
        else
        {
            m_orientationFilter->TimeUpdate(gyros, m_current_time);
            m_orientationFilter->MeasurementUpdate(acceleration, validKinematics, orientation);
            // Set orientation
            orientation[0] = m_orientationFilter->getMean(OrientationUKF::rollAngle);
            orientation[1] = m_orientationFilter->getMean(OrientationUKF::pitchAngle);
            orientation[2] = 0.0f;
            m_data->set(NUSensorsData::Orientation, m_current_time, orientation);
            // Set gyro offset values
            gyroOffset[0] = m_orientationFilter->getMean(OrientationUKF::rollGyroOffset);
            gyroOffset[1] = m_orientationFilter->getMean(OrientationUKF::pitchGyroOffset);
            gyroOffset[2] = 0.0f;
            m_data->set(NUSensorsData::GyroOffset, m_current_time, gyroOffset);
        }
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
    vector<float> orientation;
    float headYaw, headPitch;

    vector<float> supportLegTransformFlat;
    bool validKinematics = m_data->get(NUSensorsData::SupportLegTransform, supportLegTransformFlat);
    Matrix supportLegTransform = Matrix4x4fromVector(supportLegTransformFlat);
    if(validKinematics)
        orientation = Kinematics::OrientationFromTransform(supportLegTransform);

    validKinematics &= m_data->getPosition(NUSensorsData::HeadYaw, headYaw);
    validKinematics &= m_data->getPosition(NUSensorsData::HeadPitch, headPitch);
    int camera = m_camera_number;


    if (validKinematics)
    {
        HorizonLine.Calculate(orientation[1], orientation[0], headYaw, headPitch, camera);
        vector<float> line;
        line.reserve(3);
        line.push_back(HorizonLine.getA());
        line.push_back(HorizonLine.getB());
        line.push_back(HorizonLine.getC());
        m_data->set(NUSensorsData::Horizon, m_current_time, line);
    }
}


/*! @brief Calculates the duration of the last press on each of the buttons and bumpers
 */
void NUSensors::calculateButtonDurations()
{
    static float nextMainDuration = 0;
    static float nextLeftDuration = 0;
    static float nextRightDuration = 0;
    
    static float prevMainState = 0;
    static float prevLeftState = 0;
    static float prevRightState = 0;
    
	float buttonvalue;
    if (m_data->get(NUSensorsData::MainButton, buttonvalue))
    {
        if (buttonvalue > 0.5)
            nextMainDuration += (m_current_time - m_previous_time);
        else if (prevMainState > 0.5)
        {   // if this is a negative edge update the last press duration
            m_data->modify(NUSensorsData::MainButton, NUSensorsData::DurationId, m_current_time, nextMainDuration);
            nextMainDuration = 0;
        }
        prevMainState = buttonvalue;
    }
    
    if(m_data->get(NUSensorsData::LeftButton, buttonvalue))
    {
        if (buttonvalue > 0.5)
            nextLeftDuration += (m_current_time - m_previous_time);
        else if (prevLeftState > 0.5)
        {   // if this is a negative edge update the last press duration
            m_data->modify(NUSensorsData::LeftButton, NUSensorsData::DurationId, m_current_time, nextLeftDuration);
            nextLeftDuration = 0;
        }
        prevLeftState = buttonvalue;
    }
    
    if (m_data->get(NUSensorsData::RightButton, buttonvalue))
    {
        if (buttonvalue > 0.5)
            nextRightDuration += (m_current_time - m_previous_time);
        else if (prevRightState > 0.5)
        {   // if this is a negative edge update the last press duration
            m_data->modify(NUSensorsData::RightButton, NUSensorsData::DurationId, m_current_time, nextRightDuration);
            nextRightDuration = 0;
        }
        prevRightState = buttonvalue;
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
    This function should be called after calculateOrientation and calculateZMP so that those results are available.
 
    Except in special circumstances, a fall is always preceded by a falling. We transition into the falling state
    when no control action can prevent a fall. The falling is completed after the robot has impacted with the ground, 
    the impact is detected using the accelerometers, and once the robot has settled it is fallen and can start getting up.
 */
void NUSensors::calculateFallSense()
{
    static const float Fallen = 1.0;
    static const float Falling = 1.0;
    static const float FallenThreshold = 1.1;  
    static const float RollFallingThreshold = 0.55;
    static const float ForwardFallingThreshold = 0.55;
    static const float BackwardFallingThreshold = 0.45;
    static float fallen_time = 0;
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateFallingSense()" << endl;
#endif
    
    vector<float> acceleration;
    vector<float> orientation;
    
    if (m_data->getAccelerometer(acceleration) and m_data->getOrientation(orientation))
    {
        float acceleration_mag = sqrt(pow(acceleration[0],2) + pow(acceleration[1],2) + pow(acceleration[2],2));
        // check if the robot has fallen over
        vector<float> fallen(5,0);
        if (fabs(acceleration_mag - 981) < 0.2*981 and (fabs(orientation[0]) > FallenThreshold or fabs(orientation[1]) > FallenThreshold))
            fallen_time += m_current_time - m_previous_time;
        else
            fallen_time = 0;
        
        if (fallen_time > 100)
        {   // To make this sensor robust to orientation sensors that fail when the robot rolls over after falling
            // We use only the angle to determine we have fallen, not the direction
            fallen[0] = Fallen;
            if (fabs(acceleration[0]) > fabs(acceleration[1]))
            {   
                if (acceleration[0] > 0)
                {
                    fallen[3] = Fallen;     // Front
                }
                else
                {
                    fallen[4] = Fallen;     // Back
                }
            }
            else
            {
                if (acceleration[1] > 0)
                {
                    fallen[1] = Fallen;     // Left
                }
                else
                {
                    fallen[2] = Fallen;     // Right
                }
            }
        }
        m_data->set(NUSensorsData::Fallen, m_current_time, fallen);
    
        // check if the robot is falling over
        vector<float> falling(5,0);
        if (not fallen[0])
        {
            if (orientation[0] < -RollFallingThreshold)
                falling[1] = Falling;
            if (orientation[0] > RollFallingThreshold)
                falling[2] = Falling;
            if (orientation[1] > ForwardFallingThreshold)
                falling[3] = Falling;
            if (orientation[1] < -BackwardFallingThreshold)
                falling[4] = Falling;
        }
        falling[0] = falling[1] + falling[2] + falling[3] + falling[4];
        m_data->set(NUSensorsData::Falling, m_current_time, falling);
    }
    else
    {
        m_data->setAsInvalid(NUSensorsData::Fallen);
        m_data->setAsInvalid(NUSensorsData::Falling);
    }
}

/*! @brief Updates the odometry data for the current motion frame.

    This function is a wrapper for the OdometryEstimator class, retrieveing the sensor data required,
    then writing the results back as new sensor data.
 */
void NUSensors::calculateOdometry()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateOdometry()" << endl;
#endif

    // Retrieve the current odometry data.
    vector<float> odometeryData;
    m_data->getOdometry(odometeryData);
    if(odometeryData.size() < 3) odometeryData.resize(3,0.0); // Make sure the data vector is of the correct size.

    // Retrieve the foot pressures.

    float leftForce, rightForce;
    bool force_sensors = true;
    force_sensors = force_sensors && m_data->getForce(NUSensorsData::LFoot,leftForce);
    force_sensors = force_sensors && m_data->getForce(NUSensorsData::RFoot,rightForce);

    if(!force_sensors)
    {
        leftForce = -1.0f;
        rightForce = -1.0f;
    }

    // Retrieve the foot positions.
    bool validFootPosition = true;
    std::vector<float> leftPosition;
    std::vector<float> rightPosition;
    validFootPosition &= m_data->getEndPosition(NUSensorsData::LFoot, leftPosition);
    validFootPosition &= m_data->getEndPosition(NUSensorsData::RFoot, rightPosition);
    if(!validFootPosition)
        return; // If the leg transforms cannot be obtained, then no calculations can be done.

    vector<float> gpsData;
    float compassData;
    m_data->getGps(gpsData);
    m_data->getCompass(compassData);

    vector<float> odom = m_odometry->CalculateNextStep(leftPosition,rightPosition,leftForce,rightForce,gpsData,compassData);

    // Calculate differences in the position of the support leg.
    float deltaX = odom[0];
    float deltaY = odom[1];
    float deltaTheta = odom[2];

    odometeryData[0] += deltaX;
    odometeryData[1] += deltaY;
    odometeryData[2] += deltaTheta;

#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "Odometry This Frame: (" << deltaX << "," << deltaY << "," << deltaTheta << ")" << endl;
#endif        
    m_data->set(NUSensorsData::Odometry,m_data->GetTimestamp(),odometeryData);
    return;
}

/*! @brief Updates the kinematic data for the current motion frame.

    This function uses the current kinematic model and calculates the transform for each of the effectors.
    More advanced combined transforms are calculated for the current support foot and the current transfrom from the ground to the camera.
 */
void NUSensors::calculateKinematics()
{
    const double time = m_data->CurrentTime;

    // Note that the kinematics uses the Matrix class, however, at this stage the NUSensorsData stores vector<float> and vector<vector<float>>
    // In this early version we continue to use the method used in 2010:
    //		- Matrices are stored in NUSensorsData as flattened vector<float> using Matrix.asVector()
    //		- Matrices are then loaded from NUSensorsData into a temporary vector<float> then a Matrix is constructed from it using Matrix4x4fromVector
        // There is no doubt this is messy, however, meh

    // First get the joint data in the order required by the kinematic model. And find the resulting transform
    for (vector<KinematicMap>::iterator eff_it = m_kinematics_map.begin(); eff_it != m_kinematics_map.end(); ++eff_it)
    {
        // For each actuator, get the joint values
        std::vector<float> joint_positions;
        joint_positions.clear();
        float temp;
        Matrix result(4,4,false);
        for(vector<const NUData::id_t*>::iterator joint_it = eff_it->joints.begin(); joint_it != eff_it->joints.end(); ++joint_it)
        {
            if(m_data->getPosition(*(*joint_it),temp))
            {
                joint_positions.push_back(temp);
            }
            else
            {
                debug << "NUSensors::calculateKinematics(). WARNING: Unable to get position of joint: " << (*joint_it) << endl;
                errorlog << "NUSensors::calculateKinematics(). WARNING: Unable to get position of joint: " << (*joint_it) << endl;
                break;  // no use going further with this effector.
            }
        }
        if(joint_positions.size() == eff_it->joints.size())
        {
            m_kinematicModel->UpdateEffector(eff_it->index, joint_positions);
//            result = m_kinematicModel->CalculateTransform(eff_it->index, joint_positions);
            result = m_kinematicModel->EndEffectorPosition(eff_it->index);
            m_data->set(*eff_it->transform_id, time, result.asVector());
            // If the effectors position and orientation are kept, write modify them here.
            if(eff_it->effector_id)
            {
                m_data->modify(*eff_it->effector_id, NUSensorsData::EndPositionXId, time, Kinematics::PositionFromTransform(result));
                m_data->modify(*eff_it->effector_id, NUSensorsData::EndPositionRollId, time, Kinematics::OrientationFromTransform(result));
            }

        }
        else
        {
            m_data->setAsInvalid(*eff_it->transform_id);
            debug << "NUSensors::calculateKinematics(). WARNING: Incorrect number of joints: " << eff_it->transform_id << ": " << eff_it->joints.size() << " Required, " << joint_positions.size() << " Found." << endl;
            errorlog << "NUSensors::calculateKinematics(). WARNING: Incorrect number of joints: " << eff_it->transform_id << ": " << eff_it->joints.size() << " Required, " << joint_positions.size() << " Found." << endl;
        }
    }


    // This next part calculates the more complex transform requried.
    Matrix supportLegTransform;     // transform from torso to leg.
    bool legTransform = false;      // flag used to indicate the required leg transfrom was available.
    std::vector<float> temp;        // temp buffer used to store the vectorised matrices.

    bool leftFootSupport = false, rightFootSupport = false;     // flags to indicate if each foot is providing support.

    // Get the support data.
    bool validsupportdata = m_data->getSupport(NUSensorsData::LLeg,leftFootSupport);
    validsupportdata &= m_data->getSupport(NUSensorsData::RLeg,rightFootSupport);

    
    bool leftSupport = false;
    bool rightSupport = false;
    if(validsupportdata)    // if the support data was successfully retrieved.
    {
        leftSupport = leftFootSupport;
        rightSupport = rightFootSupport;
    }
    else    // Use foot height information.
    {
        bool data_ok;
        float leftHeight = 0;
        float rightHeight = 0;
        data_ok = m_data->get(NUSensorsData::LLegTransform, temp);
        if(data_ok) leftHeight = Matrix4x4fromVector(temp)[2][3];
        data_ok = m_data->get(NUSensorsData::RLegTransform, temp);
        if(data_ok) rightHeight = Matrix4x4fromVector(temp)[2][3];
        if(rightHeight < leftHeight)
        {
            leftSupport = false;
            rightSupport = true;
        }
        else
        {
            leftSupport = true;
            rightSupport = false;
        }
    }
    
    if(leftSupport || rightSupport)
    {
        // if left foot cannot be used use right
        if(!leftSupport && rightSupport)
        {
            legTransform = m_data->get(NUSensorsData::LLegTransform, temp);
        }
        // otherwise use left
        else if(leftSupport)
        {
            legTransform = m_data->get(NUSensorsData::RLegTransform, temp);
        }

        // if the leg transform was successfully retrieved.
        if(legTransform)
        {
            supportLegTransform = Matrix4x4fromVector(temp);    // convert from vector to matrix
            m_data->set(NUSensorsData::SupportLegTransform, time, temp);    // set the support leg transform to the one retrieved.
            if(m_data->get(NUSensorsData::CameraTransform, temp))   // get camera transform
            {
                // if successful
                Matrix cameraToGroundTransform = Kinematics::CalculateCamera2GroundTransform(supportLegTransform, Matrix4x4fromVector(temp));  // calculate complete transform
                m_data->set(NUSensorsData::CameraToGroundTransform, time, cameraToGroundTransform.asVector());  // set the new transform
            }
            else
            {
                m_data->setAsInvalid(NUSensorsData::CameraToGroundTransform);   // if camera not retrived mark the camera to ground transform as invalid.

            }
        }
        else
        {
            // if the required foot was not available, need to set the support leg and camera to ground transform to invalid.
            m_data->setAsInvalid(NUSensorsData::SupportLegTransform);
            m_data->setAsInvalid(NUSensorsData::CameraToGroundTransform);
        }

    }
    return;
}

void NUSensors::calculateCameraHeight()
{
//! TODO: most cases where camera height is used should really use head height at the pivot point
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensors::calculateCameraHeight()" << endl;
#endif
    vector<float> cgtvector;
    if (m_data->get(NUSensorsData::CameraToGroundTransform, cgtvector))
    {
        Matrix cameraGroundTransform = Matrix4x4fromVector(cgtvector);
        m_data->set(NUSensorsData::CameraHeight, m_data->CurrentTime, static_cast<float>(cameraGroundTransform[2][3]));
    }
    else
    {
        m_data->setAsInvalid(NUSensorsData::CameraHeight);
    }
}



