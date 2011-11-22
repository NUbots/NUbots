#ifndef SELFMODEL_H
#define SELFMODEL_H
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Moment.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Self.h"

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
    void setActive(bool newActive = true) {m_active = newActive;}

    // Alpha measure
    float alpha() const {return m_alpha;}
    void setAlpha(float newAlpha) {m_alpha = newAlpha;}

    // Model and decision tracking stuff
    unsigned int id() const {return m_id;}
    unsigned int parentid() const {return m_parent_id;}
    unsigned int splitOption() const {return m_split_option;}
    double creationTime() const {return m_creation_time;}

private:
    bool m_active;                  //!< Active flag.
    float m_alpha;                  //!< Alpha rating of the model.
    unsigned int m_id;              //!< Unique id of the model.
    unsigned int m_parent_id;       //!< Unique id of the parent model.
    unsigned int m_split_option;    //!< Option used for split from parent.
    double m_creation_time;         //!< Time at which model was created.

    /*! @brief Static function used to generate a unique incremental id for each individual model created.
    */
    static unsigned int GenerateId()
    {
        static unsigned int id = 0;
        return id++;
    }

};

#endif // SELFMODEL_H
