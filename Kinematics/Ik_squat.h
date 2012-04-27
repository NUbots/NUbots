/*! @file Ik_squat.h
    @brief Declaration of squatting cyclic inverse kinematic motion.

    @class Ik_squat
    @brief Inverse kinematics based squatting motion.
    The robot squats up and down while keeping its body level.

    @author Steven Nicklin

    Copyright (c) 2012 Steven Nicklin

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
#ifndef IK_SQUAT_H
#define IK_SQUAT_H

#include "Ik_motion.h"
#include "Tools/Math/TransformMatrices.h"
#include <iostream>

class Ik_squat: public Ik_motion
{
public:
    Ik_squat():Ik_motion(){}

    /*!
      Initialise the motion with attributes of the motion.
      @param max_height The height of the hips at the highest point of the motion, in mm.
      @param min_height The height of the hips at the highest point of the motion, in mm.
      @param stance_width The distance between the feet while performing the motion, in mm.
      @param foot_angle The angle between the feet. Measured as the angle from the right foot to the left foot, in radians.
      */
    void initialiseSquat(float max_height, float min_height, float stance_width, float foot_angle)
    {
        m_max_height = max_height;
        m_min_height = min_height;
        m_stance_width = stance_width;
        m_foot_angle = foot_angle;
    }

    // Functions to get pose for each of the limbs.
    /*!
      Returns the desired head position at the current time.
      Implementation of virtual function.
      @return In this motion the head is not used, therefore a blank Matrix is returned.
      */
    Matrix headPose()
    {
        return Matrix();
    }

    /*!
      Returns the target left foot position at the current time.
      Implementation of virtual function.
      @return The 3D transformation matrix defining the target position of the left foot
      relative to the torso coordinate system.
      */
    Matrix leftFootPose()
    {
        return footPose(true);
    }

    /*!
      Returns the target right foot position at the current time.
      Implementation of virtual function.
      @return The 3D transformation matrix defining the target position of the right foot
      relative to the torso coordinate system.
      */
    Matrix rightFootPose()
    {
        return footPose(false);
    }

    /*!
      Returns the target left hand position at the current time.
      Implementation of virtual function.
      @return In this motion the left hand is not used, therefore a blank Matrix is returned.
      */
    Matrix leftHandPose()
    {
        return Matrix();
    }

    /*!
      Returns the target right hand position at the current time.
      Implementation of virtual function.
      @return In this motion the right hand is not used, therefore a blank Matrix is returned.
      */
    Matrix rightHandPose()
    {
        return Matrix();
    }

protected:
    float m_max_height;     //! The height of the hips at the maximum point of the cycle, in mm.
    float m_min_height;     //! The height of the hips at the minimum point of the cycle, in mm.
    float m_foot_angle;     //! The angle as measured from the right foot to the left foot, in radians.
    float m_stance_width;   //! The width between the feet, in mm.

    /*!
      Calculates the target height of the hips at the crrent point of the cycle.
      @return The target height.
      */
    float currentHeight()
    {
        float prog_time = m_current_time_ms - m_cycle_start_time_ms;
        float progress = prog_time / m_period_ms;
        // using linear interpolation
        float start_height = 0;
        float end_height = 0;
        // going down
        if(progress <= 0.5)
        {
            progress *= 2;
            start_height = m_max_height;
            end_height = m_min_height;
        }
        // going back up
        else
        {
            progress = 2*(progress - 0.5);
            start_height = m_min_height;
            end_height = m_max_height;
        }
        float height = start_height + progress * (end_height - start_height);
        return height;
    }

    /*!
      Calculates the pose for a single foot.
      @param isLeft True if calculatin the left foot. False if not. If the foot is the right foot,
        the y translation and the angle of the foot are both negated.
      @return The 3D transformation matrix defining the target position for the foot.
      */
    Matrix footPose(bool isLeft)
    {
        const float mult = isLeft?0.5:-0.5;
        const float half_stance = m_stance_width * mult;
        const float half_angle = m_foot_angle * mult;
        float height = currentHeight();
        Matrix pose = TransformMatrices::Translation(0.0, half_stance, -height) * TransformMatrices::RotZ(half_angle);
        return pose;
    }
};

#endif // IK_MOTION_H
