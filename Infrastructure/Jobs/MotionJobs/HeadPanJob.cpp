/*! @file HeadPanJob.cpp
    @brief Implementation of HeadPanJob class

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

#include "HeadPanJob.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/MobileObject.h"
#include "Infrastructure/FieldObjects/StationaryObject.h"

#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a HeadPanJob
    @param pantype the type of pan (Ball, BallAndLocalisation, Localisation)
 */
HeadPanJob::HeadPanJob(head_pan_t pantype) : MotionJob(Job::MOTION_PAN)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::HeadPanJob()" << endl;
    #endif
    m_job_time = 0;
    m_pan_type = pantype;
    m_use_default = true;
    #if DEBUG_JOBS_VERBOSITY > 0
        summaryTo(debug);
    #endif
}

/*! @brief Constructs a PanJob over the given area on the field
    @param xmin the minimum x distance to include in the pan
    @param xmax the maximum x distance to include in the pan (cm)
    @param yawmin the minimum yaw angle to include in the pan (rad)
    @param yawmax the maximum yaw angle to include in the pan (rad)
 */
HeadPanJob::HeadPanJob(head_pan_t pantype, float xmin, float xmax, float yawmin, float yawmax) : MotionJob(Job::MOTION_PAN)
{
    m_job_time = 0;
    m_pan_type = pantype;
    
    m_use_default = false;
    m_x_min = xmin;
    m_x_max = xmax;
    m_yaw_min = yawmin;
    m_yaw_max = yawmax;
}

/*! @brief Constructs a PanJob to 'find' a given mobile field object, aka the ball
    @param object the mobile field object you are looking for
    @param hackfactor a scaling factor applied to the sd on the ball; this enables a manual control on the width on the pan
 */
HeadPanJob::HeadPanJob(const MobileObject& object, float hackfactor) : MotionJob(Job::MOTION_PAN)
{
    m_job_time = 0;
    m_pan_type = Ball;
    
    m_use_default = false;
    float d = object.estimatedDistance()*cos(object.estimatedElevation());
    float t = object.estimatedBearing();
    float sd = hackfactor*max(object.sdX(), object.sdY());
    
    m_x_min = d - sd;
    m_x_max = d + sd;
    m_yaw_min = t - atan2(sd,d);
    m_yaw_max = t + atan2(sd,d);
}

/*! @brief Constructs a PanJob to 'find' a given mobile field object, aka the ball
 @param object the mobile field object you are looking for
 */
HeadPanJob::HeadPanJob(const StationaryObject& object, float hackfactor) : MotionJob(Job::MOTION_PAN)
{
    m_job_time = 0;
    m_pan_type = BallAndLocalisation;
    
    m_use_default = false;
    float d = Blackboard->Objects->self.CalculateDistanceToStationaryObject(object);
    float t = Blackboard->Objects->self.CalculateBearingToStationaryObject(object);
    float sd = max(Blackboard->Objects->self.sdX(), Blackboard->Objects->self.sdY());
    float sd_t = hackfactor*2.0*Blackboard->Objects->self.sdHeading();
    
    m_x_min = d - sd;
    m_x_max = 9000;
    m_yaw_min = t - atan2(sd,d) - sd_t;
    m_yaw_max = t + atan2(sd,d) + sd_t;
}

/*! @brief Constructs a PanJob to 'find' a given mobile field object, aka the ball
    @param object the mobile field object you are looking for
 */
HeadPanJob::HeadPanJob(const vector<StationaryObject>& objects, float hackfactor) : MotionJob(Job::MOTION_PAN)
{
    m_job_time = 0;
    m_pan_type = BallAndLocalisation;
    m_use_default = false;
    
    m_x_min = 9000;
    m_x_max = 9000;
    m_yaw_min = 3.141;
    m_yaw_max = -3.141;
    for (size_t i=0; i<objects.size(); i++)
    {
        float d = Blackboard->Objects->self.CalculateDistanceToStationaryObject(objects[i]);
        float t = Blackboard->Objects->self.CalculateBearingToStationaryObject(objects[i]);
        float sd = max(Blackboard->Objects->self.sdX(), Blackboard->Objects->self.sdY());
        float sd_t = hackfactor*4.0*Blackboard->Objects->self.sdHeading();
        
        float x_min = d - sd;
        float yaw_min = t - atan2(sd,d) - sd_t;
        float yaw_max = t + atan2(sd,d) + sd_t;
        
        if (x_min < m_x_min)
            m_x_min = x_min;
        if (yaw_min < m_yaw_min)
            m_yaw_min = yaw_min;
        if (yaw_max > m_yaw_max)
            m_yaw_max = yaw_max;
    }
}

/*! @brief Constructs a HeadPanJob from stream data
    @param input the stream from which to read the job specific data
 */
HeadPanJob::HeadPanJob(istream& input) : MotionJob(Job::MOTION_PAN)
{
    m_job_time = 0;
    // Temporary read buffers
    head_pan_t hptBuffer;
    bool boolBuffer;
    float floatBuffer;

    input.read(reinterpret_cast<char*>(&hptBuffer), sizeof(hptBuffer));
    m_pan_type = hptBuffer;
    input.read(reinterpret_cast<char*>(&boolBuffer), sizeof(boolBuffer));
    m_use_default = boolBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_x_min = floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_x_max = floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_yaw_min = floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_yaw_max = floatBuffer;
    
    #if DEBUG_JOBS_VERBOSITY > 0
        summaryTo(debug);
    #endif
}

/*! @brief HeadPanJob destructor
 */
HeadPanJob::~HeadPanJob()
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::~HeadPanJob()" << endl;
    #endif
}

/*! @brief Returns the type of pan
 */
HeadPanJob::head_pan_t HeadPanJob::getPanType()
{
    return m_pan_type;
}

/*! @brief Returns true if the default values for this pan type should be used
 */
bool HeadPanJob::useDefaultValues()
{
    return m_use_default;
}

/*! @brief Gets the x distances to include in the pan */
void HeadPanJob::getX(float& xmin, float& xmax)
{
    xmin = m_x_min;
    xmax = m_x_max;
}

/*! @brief Gets the yaw angles to include in the pan */
void HeadPanJob::getYaw(float& yawmin, float& yawmax)
{
    yawmin = m_yaw_min;
    yawmax = m_yaw_max;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void HeadPanJob::summaryTo(ostream& output)
{
    output << "HeadPanJob: ";
    if (m_pan_type == Ball)
        output << "Ball";
    else if (m_pan_type == BallAndLocalisation)
        output << "BallAndLocalisation";
    else
        output << "Localisation";
    if (not m_use_default)
        output << " [" << m_x_min << "," << m_x_max << "] [" << m_yaw_min << "," << m_yaw_max << "]";
    output << endl;
    
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void HeadPanJob::csvTo(ostream& output)
{
    output << "HeadPanJob: ";
    if (m_pan_type == Ball)
        output << "Ball";
    else if (m_pan_type == BallAndLocalisation)
        output << "BallAndLocalisation";
    else
        output << "Localisation";
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
 This function calls its parents versions of the toStream, each parent
 writes the members introduced at that level
 
 @param output the stream to write the job to
 */
void HeadPanJob::toStream(ostream& output) const
{
#if DEBUG_JOBS_VERBOSITY > 1
    debug << "HeadPanJob::toStream" << endl;
#endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write HeadPanJob specific data
    output.write((char*) &m_pan_type, sizeof(m_pan_type));
    output.write((char*) &m_use_default, sizeof(m_use_default));
    output.write((char*) &m_x_min, sizeof(m_x_min));
    output.write((char*) &m_x_max, sizeof(m_x_max));
    output.write((char*) &m_yaw_min, sizeof(m_yaw_min));
    output.write((char*) &m_yaw_max, sizeof(m_yaw_max));
}

/*! @relates HeadPanJob
    @brief Stream insertion operator for a HeadPanJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadPanJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::operator<<" << endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates HeadPanJob
    @brief Stream insertion operator for a pointer to HeadPanJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadPanJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::operator<<" << endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}

