/*! @file PIDControll.cpp
 @brief Implementation of generic PID Controller
 
 @class PIDController
 @brief A generic pid controller
 
 @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <string>

class PIDController
{
public:
    PIDController(std::string name, float Kp, float Ki, float Kd, float td, float outputlowerlimit, float outputupperlimit);
    ~PIDController();
    
    void set(float target);
    float doIt(double time, float input);
    
private:
    float limitOutput(float unlimitedoutput);       //<! Clip the controller output to the actuator limits
    
private:
    std::string m_name;                             //!< The name of the controller (used to idenitify it amoung other controllers)
    
    // PID Controller constants
    float m_Kp, m_Ki, m_Kd, m_td;                   //!< The proportional, integral and derivative gains
    float m_a_constant;								//!< anti-windup constant
    float m_outputupperlimit, m_outputlowerlimit;   //!< The upper and lower limits of the controller output    
    
    // PID Controller variables
    double m_current_time, m_previous_time;         //!< The current and previous doControl times
    float m_target;                                 //!< The current set point
    float m_error, m_previous_error;                //!< The error between the set point and the actual value
    float m_proportional;   						//!< The proportional term
    double m_integral, m_previous_integral;         //!< The integral term
    float m_derivative, m_previous_derivative;      //!< The derivative term
};


#endif
