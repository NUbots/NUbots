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

#include "PIDController.h"

#include <cmath>
#include <iostream>
using namespace std;

/*! @brief Create a generic PID Controller
    @param name                   the name of the PID controller (this is used to idenitify it)
    @param Kp                     the proportional gain
    @param Ki                     the integra gain
    @param Kd                     the derivative gain
    @param period                 the control loop period
    @param outputlowerlimit       the actuator lower limit
    @param outputupperlimit       the actuator upper limit
 */
PIDController::PIDController(std::string name, float Kp, float Ki, float Kd, float outputlowerlimit, float outputupperlimit)
{
    m_name = name;
    m_Kp = Kp; 
    m_Ki = Ki;
    m_Kd = Kd;
    m_outputlowerlimit = outputlowerlimit;
    m_outputupperlimit = outputupperlimit;
    
    m_target = 0;
    
    m_time = 0;
    m_previoustime = 0;
    m_error = 0;
    m_previouserror = 0;
    m_proportional = 0;
    m_previousproportional = 0;
    m_integral = 0;
    m_previousintegral = 0;
    m_derivative = 0;
    m_previousderivative = 0;
    
    m_d_constant = 10*m_Kd;
    if (m_Kd != 0)
        m_a_constant = sqrt(Ki/Kd);
    else
        m_a_constant = 0.5*(Ki/Kp);
    
    if (m_a_constant > 1)
        m_a_constant = 1.0;
}

/*! @brief Destroys the PID controller
 */
PIDController::~PIDController()
{
}
 
/*! @brief The controller function; call this inside the control loop. Give it the measure value, it will return the new target
    @param time the current time
    @param input the measured value
    @return the calculated control output
 */
float PIDController::doControl(double time, float input)
{
    float unlimitedoutput, output;
    m_time = time;
    m_error = m_target - input;
    
    m_proportional = m_Kp*m_error;
    m_derivative = (m_Kd*(m_error - m_previouserror) + m_d_constant*m_previousderivative)/((m_time - m_previoustime) + m_d_constant);
    
    unlimitedoutput = m_proportional + m_previousintegral + m_derivative;
    output = limitOutput(unlimitedoutput);
    
    // calculate the integral term with anti-windup
    m_integral = m_integral + m_Ki*(m_time - m_previoustime)*m_error + m_a_constant*(output - unlimitedoutput); 

    // update controller state
    m_previoustime = m_time;
    m_previouserror = m_error;
    m_previousproportional = m_proportional;
    m_previousintegral = m_integral;
    m_previousderivative = m_derivative;
    return output;
}

/*! @brief Set the controller's set point
    @param ptarget the target new target
 */
void PIDController::setTarget(float ptarget)
{
    m_target = ptarget;
    return;
}

/*! @brief Clear the controller's state.
 
 This will reset the integral and derivative terms, as well as reset any historical values.
 Use this just prior to turning the motor back on, or when turning the motor off
 */
void PIDController::clearState()
{
    m_integral = 0;
    m_previousproportional = 0;
    m_previousderivative = 0;
    m_previousintegral = 0;
    m_previouserror = 0;
    return;
}
        

float PIDController::limitOutput(float unlimitedoutput)
{
    if (unlimitedoutput > m_outputupperlimit)
        return m_outputupperlimit;
    if (unlimitedoutput < m_outputlowerlimit)
        return m_outputlowerlimit;
    return unlimitedoutput;
}
