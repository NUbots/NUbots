#ifndef SELFMODEL_H
#define SELFMODEL_H
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Moment.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Self.h"
#include "WeightedModel.h"
#include <iostream>
#include <assert.h>

class OdometryMotionModel;
class MeasurementError;

/*!
  * A class used to template a self localisation kalman filter.
  * The model uses three states to track the pose, x  position, y position, and heading angle
  */
class SelfModel: public Moment, public WeightedModel
{
public:
    enum state
    {
        states_x,
        states_y,
        states_heading,
        states_total
    };

    enum updateResult
    {
        RESULT_OK,
        RESULT_OUTLIER,
        RESULT_FAILED
    };

    SelfModel(float time);
    SelfModel(const SelfModel& source);
    SelfModel(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, float time);
    virtual ~SelfModel()
    {
    }

    Matrix CalculateMeasurementPrediction(float xLocation, float yLocation) const;
    Matrix CalculateMeasurementPrediction(float xLocation, float yLocation, float orientation) const;

    static Matrix CalculateMeasurementPrediction(const Matrix& state, float xLocation, float yLocation);
    static Matrix CalculateMeasurementPrediction(const Matrix& state, float xLocation, float yLocation, float orientation);

    // Update functions
    virtual updateResult TimeUpdate(const std::vector<float>& odometry, OdometryMotionModel& motion_model, float deltaTime)
    {
        assert(false);
        return RESULT_FAILED;
    }
    virtual updateResult MultipleObjectUpdate(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement)
    {
        assert(false);
        return RESULT_FAILED;
    }
    virtual updateResult MeasurementUpdate(const StationaryObject& object, const MeasurementError& error)
    {
        assert(false);
        return RESULT_FAILED;
    }
    virtual updateResult MeasurementUpdate(const std::vector<StationaryObject*>& objects)
    {
        assert(false);
        return RESULT_FAILED;
    }

    bool isLost() const;
    Self GenerateSelfState() const;

    // Model and decision tracking stuff
    unsigned int splitOption() const {return m_split_option;}
    unsigned int previousSplitOption(const AmbiguousObject& theObject) const;

    bool clipState(int stateIndex, double minValue, double maxValue);

    bool operator ==(const SelfModel& b) const;
    bool operator !=(const SelfModel& b) const
    {return (!((*this) == b));}

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

    /*!
    @brief Output streaming operation.
    @param output The output stream.
    @param p_kf The source model to be streamed.
    */
    friend std::ostream& operator<< (std::ostream& output, const SelfModel& p_model);

    /*!
    @brief Input streaming operation.
    @param input The input stream.
    @param p_kf The destination model to be streamed to.
    */
    friend std::istream& operator>> (std::istream& input, SelfModel& p_model);

    std::string summary(bool brief=true) const;

protected:
    unsigned int m_split_option;    //!< Most recent option used for split from parent.
    std::vector<unsigned int> m_previous_decisions; //!< Stores the last decision for each of the ambiguous object types.

};

#endif // SELFMODEL_H
