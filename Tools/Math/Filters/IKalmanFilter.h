/*! @file IKalmanFilter.h
 @brief Declaration of IKalmanFilter interface class

 @class IKalmanFilter
 @brief Interface class for all Kalman Filter classes.

This contains the basic functions that all implementations can and must use.

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

#include <iostream>
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Moment.h"
class IKFModel;

class IKalmanFilter
{
public:
    ~IKalmanFilter(){delete m_model;}

    /*!
    @brief Time update function
    The time update function predicts the new state of the system.
    @param delta_t The elapsed time since the previous time update.
    @param measurement Any mesurements that can be used to predict the change in state.
    @param process_noise The linear process noise to be added to the estimate.
    @param measurement_noise The noise present in the measurment provided.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool timeUpdate(double delta_t, const Matrix& measurment, const Matrix& process_noise, const Matrix& measurement_noise) = 0;

    /*!
    @brief Measurement update function
    The measurement update function corrects the estimated state of the system using observed measurement/s.
    @param measurement The measured data to be used for the update.
    @param noise The noise associated with the measurement data provided.
    @param args Any additional arguments required for the measurment update.
    @param type The type of measurement.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool measurementUpdate(const Matrix& measurement, const Matrix& noise, const Matrix& args, unsigned int type) = 0;

    /*!
    @brief Initialisation function.
    Used to initialise the filters estimate.
    @param estimate The initial estimate of the filter.
    */
    virtual void initialiseEstimate(const Moment& estimate) = 0;

    /*!
    @brief Get function for the estimate.
    Retrieves the filters current best estimate for the system.
    @return The current estimate of the filter.
    */
    virtual const Moment& estimate() const = 0;

    // Weighting functions.
    virtual void enableWeighting(bool enabled = true) = 0;
    virtual bool getFilterWeight() const = 0;
    virtual void setFilterWeight(float weight) = 0;

    // Outlier filtering settings.
    virtual void enableOutlierFiltering(bool enabled = true) = 0;
    virtual void setOutlierThreshold(float new_threshold) = 0;
    virtual bool outlierFiltering() const = 0;
    virtual float outlierThreshold() const = 0;

    IKFModel* model() {return m_model;}

    bool operator ==(const IKalmanFilter& b) const
    {
        Moment estA = this->estimate();
        Moment estB = b.estimate();
        // Check estimates are equal
        if(estA != estB)
        {
            return false;
        }
    }
    bool operator !=(const IKalmanFilter& b) const
    {return (!((*this) == b));}

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

protected:
    IKFModel* m_model;
    IKalmanFilter(IKFModel* model): m_model(model){}
};
