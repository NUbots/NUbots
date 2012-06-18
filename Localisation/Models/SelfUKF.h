#ifndef SELFUKF_H
#define SELFUKF_H
#include "Infrastructure/FieldObjects/StationaryObject.h"
#include "Infrastructure/FieldObjects/AmbiguousObject.h"
#include "Infrastructure/FieldObjects/Self.h"
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

    void setMean(const Matrix& newMean);

    bool MeasurementUpdate(const StationaryObject& object, const MeasurementError& error);
    bool measurementUpdate(const Matrix& measurement, const Matrix& measurementNoise, const Matrix& measurementArgs = Matrix());
    bool measurementUpdateAngleBetweenTwoObjects(double angle, double x1, double y1, double x2, double y2, double angle_variance);

    Self GenerateSelfState() const;
    bool isLost() const;
    // Model and decision tracking stuff
    unsigned int splitOption() const {return m_split_option;}
    unsigned int previousSplitOption(const AmbiguousObject& theObject) const;

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
    unsigned int m_split_option;    //!< Most recent option used for split from parent.
    std::vector<unsigned int> m_previous_decisions; //!< Stores the last decision for each of the ambiguous object types.

};


#endif // SELFUKF_H
