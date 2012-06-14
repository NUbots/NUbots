#ifndef SELFUKF_H
#define SELFUKF_H
#include "Infrastructure/FieldObjects/StationaryObject.h"
#include "Infrastructure/FieldObjects/AmbiguousObject.h"
#include "Localisation/odometryMotionModel.h"
#include "Localisation/MeasurementError.h"
#include "Tools/Math/Filters/UKF.h"
#include "WeightedModel.h"

class SelfUKF: public UKF, public WeightedModel
{
public:

    enum state
    {
        states_x,
        states_y,
        states_heading,
        states_total
    };

    // Constructors
    SelfUKF();
    SelfUKF(double time);
    SelfUKF(const SelfUKF& source);
    SelfUKF(const SelfUKF& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time);

    bool clipState(int stateIndex, double minValue, double maxValue);
    Matrix processEquation(const Matrix& sigma_point, double deltaT, const Matrix& measurement);
    Matrix measurementEquation(const Matrix& sigma_point, const Matrix& measurementArgs);

    bool measurementUpdate(const Matrix& measurement, const Matrix& measurementNoise, const Matrix& measurementArgs = Matrix());

    /*!
    @brief Output streaming operation.
    @param output The output stream.
    @param p_kf The source kalman filter to be streamed.
    */
    friend std::ostream& operator<< (std::ostream& output, const SelfUKF& p_model);

    /*!
    @brief Input streaming operation.
    @param input The input stream.
    @param p_kf The destination kalman filter to be streamed to.
    */
    friend std::istream& operator>> (std::istream& input, SelfUKF& p_model);

};


#endif // SELFUKF_H
