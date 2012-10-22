/*! @file IKFModel.h
 @brief Declaration of IKFModel interface class

 @class IKFModel
 @brief This class is an interface template for a model used in a kalman filter.

 The IKFModel contains two equations, the process equation used to update the current predicition during the time update. Also the
 measurement equation used to calculate the predicted measurment from the estimated state of the system.

 The processEquation and measurementEquation virtual functions must be defined for each implemented model.

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
#include "Tools/Math/Matrix.h"

class IKFModel
{
public:
    /*! The process equation, this describes the transition of the estimate due to time and inputs applied.
      @param state The state determined frim the previous estimate.
      @param deltaT The elapsed time since the previous update was performed.
      @param measurement Measurment data obtained from the inputs to the system.
      @return The new updated measurement.
    */
    virtual Matrix processEquation(const Matrix& state, double deltaT, const Matrix& measurement) = 0;

    /*! The measurement equation, this is used to calculate the expected measurement given a state of the system.
      @param state The estimated state of the system.
      @param measurementArgs Additional information about the measurement.
      @return The expected measurment for the given conditions.
    */
    virtual Matrix measurementEquation(const Matrix& state, const Matrix& measurementArgs) = 0;

    virtual unsigned int totalStates() const = 0;

    /*!
    @brief Outputs a binary representation of the UKF object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    virtual std::ostream& writeStreamBinary (std::ostream& output) const = 0;

    /*!
    @brief Reads in a UKF object from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    virtual std::istream& readStreamBinary (std::istream& input) = 0;
};
