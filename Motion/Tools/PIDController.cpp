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


/*! @brief Create a generic PID Controller
 
 	Implements the following transfer function:
 
 				Ki         s
 	C(s) = Kp + -- + Kd --------
 				s       td.s + 1

 
    @param name the name of the PID controller (this is used to idenitify it)
    @param Kp the proportional gain
    @param Ki the integra gain
    @param Kd the derivative gain
 	@param td is the derivative filter time constant		
    @param outputlowerlimit the actuator lower limit (used for anti-windup)
    @param outputupperlimit the actuator upper limit (used for anti-windup)
 */
PIDController::PIDController(std::string name, float Kp, float Ki, float Kd, float td, float outputlowerlimit, float outputupperlimit)
{
    m_name = name;
    m_Kp = Kp; 
    m_Ki = Ki;
    m_Kd = Kd;
    m_td = td;
    m_outputlowerlimit = outputlowerlimit;
    m_outputupperlimit = outputupperlimit;
    
    m_target = 0;
    
    m_current_time = 0;
    m_previous_time = 0;
    m_error = 0;
    m_previous_error = 0;

    m_proportional = 0;
    m_integral = 0;
    m_previous_integral = 0;
    m_derivative = 0;
    m_previous_derivative = 0;
    
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
float PIDController::doIt(double time, float input)
{
    m_current_time = time;
    float dt = m_current_time - m_previous_time;
    
    m_error = m_target - input;
    m_proportional = m_Kp*m_error;
    
    float output = 0;
    if (dt < 200)
    {	// if dt is small then the controller state is valid
        if (m_Kd != 0 and m_td != 0)
            m_derivative = (m_Kd*(m_error - m_previous_error) + m_td*m_previous_derivative)/(m_td + m_td*dt);
        else
            m_derivative = 0;
    
        float unlimitedoutput = m_proportional + m_previous_integral + m_derivative;
        output = limitOutput(unlimitedoutput);
    
        // calculate the integral term with anti-windup
        m_integral = m_integral + m_Ki*dt*m_error + m_a_constant*(output - unlimitedoutput); 
    }
    else
    {	// if there has been a large gap in the controller action, then the state needs to be reset
        m_derivative = 0;
        m_integral = 0;
        output = limitOutput(m_proportional);
    }

    // update controller state
    m_previous_time = m_current_time;
    m_previous_error = m_error;
    m_previous_integral = m_integral;
    m_previous_derivative = m_derivative;
    return output;
}

/*! @brief Set the controller's set point
    @param ptarget the target new target
 */
void PIDController::set(float ptarget)
{
    m_target = ptarget;
}

float PIDController::limitOutput(float unlimitedoutput)
{
    if (unlimitedoutput > m_outputupperlimit)
        return m_outputupperlimit;
    if (unlimitedoutput < m_outputlowerlimit)
        return m_outputlowerlimit;
    return unlimitedoutput;
}
