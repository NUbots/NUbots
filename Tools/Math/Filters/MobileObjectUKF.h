/*! @file MobileObjectUKF.h
 @brief Declaration of MobileObjectUKF class

 @class MobileObjectUKF
 @brief This class is a UKF implementation used to track a mobile object.

 The mobile objects relative position in relation to the observer is tracked. The
 velocity of the object is also tracked.

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

#pragma once

#include "UKF.h"

class MobileObjectUKF : public UKF
{
public:
    enum State
    {
        x_pos,
        y_pos,
        x_vel,
        y_vel,
        total_states
    };

    MobileObjectUKF();
    ~MobileObjectUKF();
protected:
    Matrix processEquation(const Matrix& sigma_point, double deltaT, const Matrix& measurement);
    Matrix measurementEquation(const Matrix& sigma_point, const Matrix& measurementArgs);
    float m_velocity_decay; //! The velocity decay rate, should be <1 and >0. Velocity becomes m_velocity_decay*current velocity.
};
