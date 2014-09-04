#pragma once
#include "IKFModel.h"


class IMUModel : public IKFModel
{
public:
    enum State
    {
        kstates_gyro_offset_x,
        kstates_gyro_offset_y,
        kstates_body_angle_x,
        kstates_body_angle_y,
        kstates_total
    };

    enum MeasurementType
    {
        kmeasurement_accelerometer,
        kmeasurement_kinematic,
        kmeasurement_total_types
    };

    IMUModel();
    IKFModel* Clone()
    {
        return new IMUModel(*this);
    }

    /*! The process equation, this describes the transition of the estimate due to time and inputs applied.
      @param state The state determined frim the previous estimate.
      @param deltaT The elapsed time since the previous update was performed.
      @param measurement Measurment data obtained from the inputs to the system.
      @return The new updated measurement.
    */
    Matrix processEquation(const Matrix& state, double deltaT, const Matrix& measurement);

    /*! The measurement equation, this is used to calculate the expected measurement given a state of the system.
      @param state The estimated state of the system.
      @param measurementArgs Additional information about the measurement.
      @return The expected measurment for the given conditions.
    */
    Matrix measurementEquation(const Matrix& state, const Matrix& measurementArgs, unsigned int type);

    Matrix measurementDistance(const Matrix& measurement1, const Matrix& measurement2, unsigned int type);

    void limitState(Matrix &state);

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
    IMUModel(const IMUModel& source);
    Matrix kinematicMeasurementEquation(const Matrix& state, const Matrix& measurementArgs);
    Matrix accelerometerMeasurementEquation(const Matrix& state, const Matrix& measurementArgs);
};
