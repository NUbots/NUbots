/*! @file SeqUKF.h
 @brief Declaration of general SeqUKF class

 @class SeqUKF
 @brief This class is a template to create a custom SeqUKF specific for an application.

 The SeqUKF is a Sequential Unscented Kalman filter, where measurement updates are able to be performed
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
#include "IKalmanFilter.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Moment.h"
#include "UnscentedTransform.h"
class SeqUKF: public IKalmanFilter
{
public:
    SeqUKF(IKFModel* model);
    SeqUKF(const SeqUKF& source);
    ~SeqUKF();

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
    void initialiseEstimate(const Moment& estimate);

    /*!
    @brief Get function for the estimate.
    Retrieves the filters current best estimate for the system.
    @return The current estimate of the filter.
    */
    const Moment& estimate() const;

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
private:

    Moment m_estimate;
    Matrix m_mean_weights;
    Matrix m_covariance_weights;
    Matrix m_sqrt_covariance_weights;
    Matrix m_sigma_points;
    Matrix m_sigma_mean;
    Matrix m_C;
    Matrix m_d;
    Matrix m_X;

    UnscentedTransform m_unscented_transform;

    // Functions for performing steps of the UKF algorithm.
    void CalculateWeights();
    Matrix GenerateSigmaPoints() const;
    Matrix CalculateMeanFromSigmas(const Matrix& sigmaPoints) const;
    Matrix CalculateCovarianceFromSigmas(const Matrix& sigmaPoints, const Matrix& mean) const;
};
