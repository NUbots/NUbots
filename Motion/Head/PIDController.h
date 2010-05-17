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
    PIDController(std::string name, float Kp, float Ki, float Kd, float outputlowerlimit, float outputupperlimit);
    ~PIDController();
    
    float doControl(double time, float input);
    void setTarget(float target);
    
    void clearState();
    
private:
    std::string m_name;                             //!< The name of the controller (used to idenitify it amoung other controllers)
    
    // PID Controller constants
    float m_Kp, m_Ki, m_Kd;                         //!< The proportional, integral and derivative gains
    float m_outputupperlimit, m_outputlowerlimit;   //!< The upper and lower limits of the controller output    
    float m_d_constant, m_a_constant;               //!< The derivative filter constant, and the anti-windup filter constant
    
    // PID Controller variables
    double m_time, m_previoustime;                  //!< The current and previous doControl times
    float m_target;                                 //!< The current set point
    float m_error, m_previouserror;                 //!< The error between the set point and the actual value
    float m_proportional, m_previousproportional;   //!< The proportional term
    double m_integral, m_previousintegral;          //!< The integral term
    float m_derivative, m_previousderivative;       //!< The derivative term
    
    float limitOutput(float unlimitedoutput);       //<! Clip the controller output to the actuator limits
};


#endif
