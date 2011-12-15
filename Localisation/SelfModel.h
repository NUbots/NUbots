#ifndef SELFMODEL_H
#define SELFMODEL_H
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Moment.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Self.h"
#include <iostream>
#include <boost/circular_buffer.hpp>

/*!
  * A class used to template a self localisation kalman filter.
  * The model uses three states to track the pose, x  position, y position, and heading angle
  */
class SelfModel: public Moment
{
public:
    enum state
    {
        states_x,
        states_y,
        states_heading,
        states_total
    };

    SelfModel(float time);
    SelfModel(const SelfModel& source);
    SelfModel(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, float time);

    Matrix CalculateMeasurementPrediction(float xLocation, float yLocation) const;
    Matrix CalculateMeasurementPrediction(float xLocation, float yLocation, float orientation) const;

    static Matrix CalculateMeasurementPrediction(const Matrix& state, float xLocation, float yLocation);
    static Matrix CalculateMeasurementPrediction(const Matrix& state, float xLocation, float yLocation, float orientation);

    bool isLost() const;
    Self GenerateSelfState() const;

    // Multiple models stuff
    bool active() const {return m_active;}
    bool inactive() const {return !m_active;}
    void setActive(bool newActive = true) {m_active = newActive;}

    // Alpha measure
    float alpha() const {return m_alpha;}
    void setAlpha(float newAlpha) {m_alpha = newAlpha;}

    // Model and decision tracking stuff
    unsigned int id() const {return m_id;}
    unsigned int parentid() const {return m_parent_id;}
    unsigned int history(unsigned int samples_back) const;
    unsigned int history_depth() const {return m_history_depth;}
    boost::circular_buffer<unsigned int> full_history() const {return m_history_buffer;}
    unsigned int splitOption() const {return m_split_option;}
    double creationTime() const {return m_creation_time;}
    unsigned int previousSplitOption(const AmbiguousObject& theObject) const;

    bool operator < (const SelfModel& model) const
    {
        return (alpha() < model.alpha());
    }

    bool operator > (const SelfModel& model) const
    {
        return (alpha() > model.alpha());
    }

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

protected:
    bool m_active;                  //!< Active flag.
    float m_alpha;                  //!< Alpha rating of the model.
    unsigned int m_id;              //!< Unique id of the model.
    unsigned int m_parent_id;       //!< Unique id of the parent model.
    unsigned int m_split_option;    //!< Most recent option used for split from parent.
    double m_creation_time;         //!< Time at which model was created.

    unsigned int m_history_depth;
    boost::circular_buffer<unsigned int> m_history_buffer;
    std::vector<unsigned int> m_previous_decisions; //!< Stores the last decision for each of the ambiguous object types.

    /*! @brief Static function used to generate a unique incremental id for each individual model created.
    */
    static unsigned int GenerateId()
    {
        static unsigned int id = 0;
        return id++;
    }

};

#endif // SELFMODEL_H
