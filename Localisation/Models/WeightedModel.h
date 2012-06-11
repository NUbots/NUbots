#pragma once

#include <iostream>
#include <assert.h>
#include <boost/circular_buffer.hpp>

/*!
  * A class used to template a weighted model.
  * The weighted models are used in multiple model filter systems, with the weighting
  * denoting the model significance.
  */
class WeightedModel
{
public:
    WeightedModel(float time);
    WeightedModel(const WeightedModel& source);
    WeightedModel(const WeightedModel& parent, float time);


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
    unsigned int history_depth() const {return m_parent_history_buffer.size();}
    boost::circular_buffer<unsigned int> full_history() const {return m_parent_history_buffer;}
    double creationTime() const {return m_creation_time;}

    bool operator < (const WeightedModel& model) const
    {
        return (alpha() < model.alpha());
    }

    bool operator > (const WeightedModel& model) const
    {
        return (alpha() > model.alpha());
    }

    std::string summary(bool brief=true) const;

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
    bool m_active;                  //!< Active flag.
    float m_alpha;                  //!< Alpha rating of the model.
    unsigned int m_id;              //!< Unique id of the model.
    unsigned int m_parent_id;       //!< Unique id of the parent model.
    double m_creation_time;         //!< Time at which model was created.

    boost::circular_buffer<unsigned int> m_parent_history_buffer;

    /*! @brief Static function used to generate a unique incremental id for each individual model created.
    */
    static unsigned int GenerateId()
    {
        static unsigned int id = 0;
        return id++;
    }
};
