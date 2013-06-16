/*! @file WSeqUKF.h
 @brief Declaration of general WSeqUKF class

 @class WSeqUKF
 @brief This class is a template to create a custom WSeqUKF specific for an application.

 The WSeqUKF is a Sequential Unscented Kalman filter, where measurement updates are able to be performed
 sequentially rather than in parallel, while still returning the identical numerical results.

 The processEquation and measurementEquation virtual functions must be defined for each application,
 the remainder of the algorithm is otherwise identical.

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
#include "IWeightedKalmanFilter.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/MultivariateGaussian.h"
#include "UnscentedTransform.h"
#include "IKalmanFilter.h"
class WSeqUKF: public IWeightedKalmanFilter
{
public:
    WSeqUKF(IKFModel* model);
    ~WSeqUKF();
    IWeightedKalmanFilter* Clone()
    {
        return new WSeqUKF(*this);
    }

    /*!
    @brief Time update function
    The time update function predicts the new state of the system.
    @param delta_t The elapsed time since the previous time update.
    @param measurement Any mesurements that can be used to predict the change in state.
    @param process_noise The linear process noise to be added to the estimate.
    @param measurement_noise The noise present in the measurment provided.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool timeUpdate(double delta_t, const Matrix& measurement, const Matrix& process_noise, const Matrix& measurement_noise);

    /*!
    @brief Measurement update function
    The measurement update function corrects the estimated state of the system using observed measurement/s.
    @param measurement The measured data to be used for the update.
    @param noise The noise associated with the measurement data provided.
    @param args Any additional arguments required for the measurment update.
    @param type The type of measurement.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool measurementUpdate(const Matrix& measurement, const Matrix& noise, const Matrix& args, unsigned int type);

    /*!
    @brief Initialisation function.
    Used to initialise the filters estimate.
    @param estimate The initial estimate of the filter.
    */
    void initialiseEstimate(const MultivariateGaussian& estimate);

    std::string summary(bool detailed) const;

    /*!
    @brief Outputs a binary representation of the UKF object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    std::ostream& writeStreamBinary (std::ostream& output) const;

    /*!
    @brief Reads in a UKF object from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    std::istream& readStreamBinary (std::istream& input);

    // Weighting functions.
    void enableWeighting(bool enabled = true) {m_weighting_enabled = enabled;}
    float getFilterWeight() const {return m_filter_weight;}
    void setFilterWeight(float weight) {m_filter_weight = weight;}

protected:

    WSeqUKF(const WSeqUKF& source);
    bool m_weighting_enabled;
    float m_filter_weight;
    Matrix m_sigma_points;
    Matrix m_sigma_mean;
    Matrix m_C;
    Matrix m_d;
    Matrix m_X;

    UnscentedTransform m_unscented_transform;

    void init();
    bool evaluateMeasurement(const Matrix& innovation, const Matrix& estimate_variance, const Matrix& measurement_variance);
};
