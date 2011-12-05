#ifndef ODOMETRY_ESTIMATOR_H
#define ODOMETRY_ESTIMATOR_H
/*! @file OdometryEstimator.h
 @brief Definition of a class used to estimate the robots odometry
 
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

#include <vector>
#include <fstream>
#include <string>

class OdometryEstimator
{
public:
    // Enum for leg selection.
    enum LegIdentifier {none, left, right};
    OdometryEstimator();
    ~OdometryEstimator();
    void IntialiseFile(std::string filename);
    void WriteLogData(std::vector<float>& gps, float compass,
                      const std::vector<float>& leftOdom, const std::vector<float>& rightOdom,
                      float leftForce, float rightForce, float leftZ, float rightZ, OdometryEstimator::LegIdentifier supportLeg);
    LegIdentifier SelectSupportLegTouch(float forceLeft, float forceRight);
    LegIdentifier SelectSupportLegKinematic(float left_z, float right_z);
    std::vector<float> CalculateNextStep(const std::vector<float>& leftPos, const std::vector<float>& rightPos,
                                         float forceLeft, float forceRight, std::vector<float>& gps, float compass);
private:
    // Logging values
    bool m_logging_enabled;
    std::fstream m_odometry_log;

    // Tuning variables
    float m_minimum_support_foot_pressure;
    float m_turn_multiplier;   // Turn Gripping factor
    float m_x_multiplier;      // X Gripping factor
    float m_y_multiplier;      // Y Gripping factor

    // Variables used in calculation of odometry values.
    LegIdentifier m_support_leg;
    std::vector<float> m_left_foot_position;
    std::vector<float> m_right_foot_position;
};

#endif
