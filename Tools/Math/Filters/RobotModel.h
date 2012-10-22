#pragma once
#include "IKFModel.h"


class RobotModel : public IKFModel
{
public:
    enum state
    {
        states_x,
        states_y,
        states_heading,
        states_total
    };

    RobotModel();

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
    Matrix measurementEquation(const Matrix& state, const Matrix& measurementArgs);

    unsigned int totalStates() const
    {
        return states_total;
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
    Matrix m_time_process_matrix;
};
