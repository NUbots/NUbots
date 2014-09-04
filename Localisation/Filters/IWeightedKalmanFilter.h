/*! @file IKalmanFilter.h
 @brief Declaration of IKalmanFilter interface class

 @class IKalmanFilter
 @brief Interface class for all Kalman Filter classes.

This contains the basic functions that all implementations can and must use.

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
#include <assert.h>
#include <iostream>
#include "Tools/Math/Matrix.h"
#include "Tools/Math/MultivariateGaussian.h"
#include "IKFModel.h"
#include <boost/circular_buffer.hpp>
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "IKalmanFilter.h"

class IWeightedKalmanFilter: public IKalmanFilter
{
public:
    virtual ~IWeightedKalmanFilter()
    {
        m_parent_history_buffer.clear();
        m_previous_decisions.clear();
    }

    virtual IWeightedKalmanFilter* Clone() = 0;

    // Active controld
    virtual bool active() const {return m_active;}
    virtual void setActive(bool active = true) {m_active = active;}
    virtual unsigned int id() const {return m_id;}
    virtual void AssignNewId() {m_id = GenerateId();}

    // Weighting functions.
    virtual void enableWeighting(bool enabled = true) = 0;
    virtual float getFilterWeight() const = 0;
    virtual void setFilterWeight(float weight) = 0;

// Multiple model stuff
    /*! @brief Get the previous decision path when this ambiguous object was last encountered.
        @param theObject The ambiguous object.
        @return The object id of the unique object that was last chosen.
    */
    unsigned int previousSplitOption(const AmbiguousObject& theObject) const
    {
        unsigned int result;
        unsigned int objectIndex = theObject.getID();
        assert(objectIndex < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
        result = m_previous_decisions[objectIndex];
        return result;
    }

    double creationTime() const {return m_creation_time;}

    /*!
    @brief Outputs a binary representation of the IWeightedKalmanFilter object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    virtual std::ostream& writeStreamBinary (std::ostream& output) const
    {

    }

    /*!
    @brief Reads in a IWeightedKalmanFilter object from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    virtual std::istream& readStreamBinary (std::istream& input)
    {

    }

    // Multiple model stuff
    unsigned int m_split_option;    //!< Most recent option used for split from parent.
    std::vector<unsigned int> m_previous_decisions; //!< Stores the last decision for each of the ambiguous object types.
    unsigned int m_parent_id;       //!< Unique id of the parent model.
    double m_creation_time;         //!< Time at which model was created.
    boost::circular_buffer<unsigned int> m_parent_history_buffer;

protected:
    // Multiple Model stuff
    bool m_active;
    unsigned int m_id;
    IWeightedKalmanFilter(IKFModel* model): IKalmanFilter(model)
    {
        m_previous_decisions.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, FieldObjects::NUM_STAT_FIELD_OBJECTS);
        m_parent_history_buffer.resize(5,0);
        m_id = GenerateId();
    }

    IWeightedKalmanFilter(const IWeightedKalmanFilter& source): IKalmanFilter(source)
    {
        m_previous_decisions = source.m_previous_decisions;
        m_parent_history_buffer = source.m_parent_history_buffer;
        m_parent_id = source.id();
        m_active = source.m_active;
        m_id = m_id;
        m_creation_time = source.m_creation_time;
    }

    static unsigned int GenerateId()
    {
        static unsigned int id = 0;
        return id++;
    }
};
