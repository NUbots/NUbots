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
#include "Tools/Math/Moment.h"
#include "IKFModel.h"
#include <boost/circular_buffer.hpp>
#include "Infrastructure/FieldObjects/FieldObjects.h"

class IKalmanFilter
{
public:
    virtual ~IKalmanFilter()
    {
        if(m_model != NULL)
            delete m_model;
        m_parent_history_buffer.clear();
        m_previous_decisions.clear();
    }

    virtual IKalmanFilter* Clone() = 0;

    /*!
    @brief Time update function
    The time update function predicts the new state of the system.
    @param delta_t The elapsed time since the previous time update.
    @param measurement Any mesurements that can be used to predict the change in state.
    @param process_noise The linear process noise to be added to the estimate.
    @param measurement_noise The noise present in the measurment provided.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool timeUpdate(double delta_t, const Matrix& measurment, const Matrix& process_noise, const Matrix& measurement_noise) = 0;

    /*!
    @brief Measurement update function
    The measurement update function corrects the estimated state of the system using observed measurement/s.
    @param measurement The measured data to be used for the update.
    @param noise The noise associated with the measurement data provided.
    @param args Any additional arguments required for the measurment update.
    @param type The type of measurement.
    @return True if the update was performed sucessfully. False if the update was unable to be performed.
    */
    virtual bool measurementUpdate(const Matrix& measurement, const Matrix& noise, const Matrix& args, unsigned int type) = 0;

    /*!
    @brief Initialisation function.
    Used to initialise the filters estimate.
    @param estimate The initial estimate of the filter.
    */
    virtual void initialiseEstimate(const Moment& estimate) = 0;

    /*!
    @brief Get function for the estimate.
    Retrieves the filters current best estimate for the system.
    @return The current estimate of the filter.
    */
    virtual const Moment& estimate() const = 0;

    // Active controld
    virtual bool active() const {return m_active;}
    virtual void setActive(bool active = true) {m_active = active;}
    virtual unsigned int id() const {return m_id;}
    virtual void AssignNewId() {m_id = GenerateId();}

    // Weighting functions.
    virtual void enableWeighting(bool enabled = true) = 0;
    virtual float getFilterWeight() const = 0;
    virtual void setFilterWeight(float weight) = 0;

    // Outlier filtering settings.
    virtual void enableOutlierFiltering(bool enabled = true) = 0;
    virtual void setOutlierThreshold(float new_threshold) = 0;
    virtual bool outlierFiltering() const = 0;
    virtual float outlierThreshold() const = 0;

    IKFModel* model() {return m_model;}

    bool operator ==(const IKalmanFilter& b) const
    {
        Moment estA = this->estimate();
        Moment estB = b.estimate();
        // Check estimates are equal
        if(estA != estB)
        {
            return false;
        }
    }
    bool operator !=(const IKalmanFilter& b) const
    {return (!((*this) == b));}

    /*!
    @brief Outputs a binary representation of the UKF object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    virtual std::ostream& writeStreamBinary (std::ostream& output) const = 0;

    /*!
    @brief Reads in a UKF object from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    virtual std::istream& readStreamBinary (std::istream& input) = 0;

    virtual std::string summary(bool detailed) const = 0;

// Multipl model stuff
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

    void setModel(IKFModel* newModel)
    {
        if(m_model) delete m_model;
        m_model = newModel;
    }

    double creationTime() const {return m_creation_time;}

    // Multiple model stuff
    unsigned int m_split_option;    //!< Most recent option used for split from parent.
    std::vector<unsigned int> m_previous_decisions; //!< Stores the last decision for each of the ambiguous object types.
    unsigned int m_parent_id;       //!< Unique id of the parent model.
    double m_creation_time;         //!< Time at which model was created.
    boost::circular_buffer<unsigned int> m_parent_history_buffer;

protected:
    IKFModel* m_model;

    // Multiple Model stuff
    bool m_active;
    unsigned int m_id;
    IKalmanFilter(IKFModel* model): m_model(model)
    {
        m_previous_decisions.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, FieldObjects::NUM_STAT_FIELD_OBJECTS);
        m_parent_history_buffer.resize(5,0);
        m_id = GenerateId();
    }

    IKalmanFilter(const IKalmanFilter& source)
    {
        m_model = source.m_model->Clone();
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
