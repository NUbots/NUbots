/*! @file DarwinJointMapping.h
    @brief Declaration of DarwinJointMapping class

    @class DarwinJointMapping
    @brief A singleton class to convert between the raw data read from the Darwin robot motors,
    and the joint angles used in the NUbots robotics software system.

    The NUbots joint space is the standard joint angle definitions constant between all robots. The
    zero position and direction of the motor angle is entierly dependant on the mounting of the motor.
    Because of this we wish to convert the angles to a common frame of reference, in this case centred on the
    origin coordinate system used for all other calculations. The right hand rule is applied so that a positive
    angle is clockwise looking in the direction of each axis. Roll is about the x-axis that is directed forward
    from the chest of the robot. Pitch is about the y-axis directed sideward to the left of the robot. Yaw is
    about the z-axis directed upward through the head of the robot.
    In order to convert the angles into this space an offset and multiplier are applied to each angle.

    The static functions Value2Radian() and Radian2Value() are a direct conversion between the raw positive integer
    value read from the Robotis MX28 motors and a motor angle in radians.

    The functions joint2raw() and raw2joint() perform the conversion between joint angle and motor position value,
    but also apply the offset and multiplier required to convert the motor angle into a standardised joint angle
    within the Nubots software. The remaning body2raw() and raw2body() functions are used to perform this function
    on the entire set of joint angles as opposed to a single joint.

    @author Steven Nicklin

    Copyright (c) 2011 Steven Nicklin

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

#ifndef DARWINJOINTMAPPING_H
#define DARWINJOINTMAPPING_H

#include <vector>
#include "Tools/Math/Limit.h"

class DarwinJointMapping
{
public:
    /*! @brief  Retrieve the single persistant instance of the DarwinJointMapping singleton class.
        @return The single persistant instance of this class.
     */
    static DarwinJointMapping& Instance()
    {
      static DarwinJointMapping singleton;
      return singleton;
    }

    int joint2raw(unsigned int id, float joint) const;
    int joint2rawClipped(unsigned int id, float joint) const;
    float raw2joint(unsigned int id, int raw) const;
    std::vector<int> body2raw(const std::vector<float>& body) const;
    std::vector<float> raw2body(const std::vector<int>& raw) const;
    static int Radian2Value(float radian);
    static float Value2Radian(int value);
protected:
    std::vector<float> m_offsets;                               //!< Table of offset values for each joint.
    std::vector<char> m_multipliers;                            //!< Table of multiplier values for each joint.
    std::vector<Limit> m_limits;                                 //!< Limit value for each joint.
    DarwinJointMapping();                                       //!< Private constructor
    DarwinJointMapping(const DarwinJointMapping&);              //!< Prevent copy-construction
    DarwinJointMapping& operator=(const DarwinJointMapping&);   //!< Prevent assignment
};

#endif // DARWINJOINTMAPPING_H
