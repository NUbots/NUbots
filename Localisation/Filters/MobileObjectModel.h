#pragma once

#include "IKFModel.h"

class MobileObjectModel : public IKFModel
{
public:

    enum State
    {
        kstates_x_pos,
        kstates_y_pos,
        kstates_x_vel,
        kstates_y_vel,
        kstates_total
    };

    enum MeasurementType
    {
        kobserved_measurement,
        kshared_measurement,
        ktotal_measurement_types
    };

    MobileObjectModel();

    IKFModel* Clone()
    {
        return new MobileObjectModel(*this);
    }

    /*! The process equation, this describes the transition of the estimate due to time and inputs applied.
      @param state The state determined frim the previous estimate.
      @param delta_t The elapsed time since the previous update was performed.
      @param measurement Measurment data obtained from the inputs to the system.
      @return The new updated measurement.
    */
    Matrix processEquation(const Matrix& state, double delta_t, const Matrix& measurement);

    /*! The measurement equation, this is used to calculate the expected measurement given a state of the system.
      @param state The estimated state of the system.
      @param measurementArgs Additional information about the measurement.
      @return The expected measurment for the given conditions.
    */
    Matrix measurementEquation(const Matrix& state, const Matrix& measurementArgs, unsigned int type);
    Matrix observedMeasurementEquation(const Matrix& state, const Matrix& measurementArgs);

    Matrix measurementDistance(const Matrix& measurement1, const Matrix& measurement2, unsigned int type);
    void limitState(Matrix& state){(void)(state);}

    unsigned int totalStates() const
    {
        return kstates_total;
    }

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

protected:
    float m_velocity_decay; //! The velocity decay rate, should be <1 and >0. Velocity becomes m_velocity_decay*current velocity.
    MobileObjectModel(const MobileObjectModel& source);
};
