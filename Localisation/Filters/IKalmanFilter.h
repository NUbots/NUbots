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
#include <assert.h>
#include <iostream>
#include "Tools/Math/Matrix.h"
#include "Tools/Math/MultivariateGaussian.h"
#include "IKFModel.h"


class IKalmanFilter
{
public:
    virtual ~IKalmanFilter()
    {
        if(m_model != NULL)
            delete m_model;
    }

    virtual IKalmanFilter* Clone() = 0;

    /*!
    @brief Time update function
    The time update function predicts the new state of the system.
    @param delta_t The elapsed time since the previous time update.
    @param measurement Any mesurements that can be used to predict the change in state.
    @param process_noise The linear process noise to be added to the estimate.
    @param measurement_noise The noise present in the measurment provided.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool timeUpdate(double delta_t, const Matrix& measurement, const Matrix& process_noise, const Matrix& measurement_noise) = 0;

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
    virtual void initialiseEstimate(const MultivariateGaussian& estimate) = 0;

    /*!
    @brief Get function for the estimate.
    Retrieves the filters current best estimate for the system.
    @return The current estimate of the filter.
    */
    virtual const MultivariateGaussian& estimate() const {return m_estimate;}

    // Outlier filtering settings.
    virtual void enableOutlierFiltering(bool enabled = true) {m_outlier_filtering_enabled = enabled;}
    virtual void setOutlierThreshold(float new_threshold){m_outlier_threshold = new_threshold;}
    virtual bool outlierFiltering() const {return m_outlier_filtering_enabled;}
    virtual float outlierThreshold() const {return m_outlier_threshold;}

    IKFModel* model() {return m_model;}

    bool operator ==(const IKalmanFilter& b) const
    {
        MultivariateGaussian estA = this->estimate();
        MultivariateGaussian estB = b.estimate();
        // Check estimates are equal
        if(estA != estB)
        {
            return false;
        }
        return true;
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

    virtual std::string summary(bool detailed) const = 0;

    void setModel(IKFModel* newModel)
    {
        if(m_model) delete m_model;
        m_model = newModel;
    }

protected:
    IKFModel* m_model;
    MultivariateGaussian m_estimate;
    bool m_outlier_filtering_enabled;
    float m_outlier_threshold;

    IKalmanFilter(IKFModel* model): m_model(model), m_estimate(model->totalStates())
    {
        m_outlier_filtering_enabled = false;
        m_outlier_threshold = 15.f;
    }

    IKalmanFilter(const IKalmanFilter& source): m_estimate(source.estimate())
    {
        m_model = source.m_model->Clone();
        m_outlier_filtering_enabled = source.m_outlier_filtering_enabled;
        m_outlier_threshold = source.m_outlier_threshold;
    }
};
