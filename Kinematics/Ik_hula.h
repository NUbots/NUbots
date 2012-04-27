/*! @file Ik_hula.h
    @brief Declaration of hula cyclic inverse kinematic motion.

    @class Ik_hula
    @brief Inverse kinematics based hula motion.
    The robot rotates in a circular motion using its hips.

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

#ifndef IK_HULA_H
#define IK_HULA_H

#include "Ik_motion.h"
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"
#include <iostream>

class Ik_hula: public Ik_motion
{
public:
    Ik_hula():Ik_motion(){}

    /*!
      Initialise the motion with attributes of the motion.
      @param height The height of the hips while performing the motion, in mm.
      @param stance_width The distance between the feet while performing the motion, in mm.
      @param foot_angle The angle between the feet. Measured as the angle from the right foot to the left foot, in radians.
      @param radius The radius of the roation motion. A larger radius leads to a more exagurated motion, in mm.
      */
    void initialiseHula(float height, float stance_width, float foot_angle, float radius)
    {
        m_height = height;
        m_stance_width = stance_width;
        m_foot_angle = foot_angle;
        m_radius = radius;
    }

    // Functions to get pose for each of the limbs.
    /*!
      Returns the target head position at the current time.
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
    float m_height;         //! The height of the hips during the motion, in mm.
    float m_stance_width;   //! The width between the legs during this motion, in mm.
    float m_foot_angle;     //! The angle from the right foot to the left foot during the motion, in radians.
    float m_radius;         //! The radius of the circular hula motion, in mm.

    /*!
      Calculates the target angle in the radial hula motion at the current time.
      @return The target angle in radians.
      */
    float current_angle()
    {
        float prog_time = m_current_time_ms - m_cycle_start_time_ms;
        float progress = prog_time / m_period_ms;
        return 2 * mathGeneral::PI * progress;
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
        float angle = current_angle();
        float x_offset = m_radius * cos(angle);
        float y_offset = m_radius * sin(angle);

        Matrix pose = TransformMatrices::Translation(-x_offset, half_stance - y_offset, -m_height) * TransformMatrices::RotZ(half_angle);
        return pose;
    }
};

#endif // IK_MOTION_H
