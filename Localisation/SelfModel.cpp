#include "SelfModel.h"
#include "Tools/Math/General.h"

/*! @brief Default time constructor

    This constructor requires a creation time.

 */
SelfModel::SelfModel(float time): Moment(states_total)
{
    m_id = GenerateId();
    m_creation_time = time;
    m_alpha = 1.0f;
    m_history_depth = 5;
    m_history_buffer = boost::circular_buffer<unsigned int>(m_history_depth);
    m_previous_decisions.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, FieldObjects::NUM_STAT_FIELD_OBJECTS);
    return;
}

/*! @brief Copy constructor

    Creates an exact copy of the original model, Preserving id, parent and splitting information.
 */
SelfModel::SelfModel(const SelfModel& source): Moment(source)
{
    m_active = source.m_active;
    m_alpha =  source.m_alpha;
    m_covariance = source.m_covariance;
    m_id = source.m_id;
    m_mean = source.m_mean;
    m_parent_id = source.m_parent_id;
    m_split_option = source.m_split_option;
    m_creation_time = source.m_creation_time;
    m_history_depth = source.m_history_depth;
    m_history_buffer = source.m_history_buffer;
    m_previous_decisions = source.m_previous_decisions;
    return;
}

/*! @brief Split constructor

Takes a parent model and performs a split on an ambiguous object using the given split option.

@param parent the parent model from which the split orignates.
@param object the ambiguous object belong evaluated by the split.
@param splitOption the option to be evaluated within this model.
@param time the current time of the update
*/
SelfModel::SelfModel(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, float time): Moment(states_total)
{
    // Save the information about the model
    m_id = GenerateId();
    m_parent_id = parent.id();
    m_history_buffer = parent.m_history_buffer; // Get parents history.
    m_history_buffer.push_back(m_parent_id);    // Add parent to the history.
    m_split_option = splitOption.getID();
    assert(object.getID() < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
    m_previous_decisions = parent.m_previous_decisions;
    m_previous_decisions[object.getID()] = splitOption.getID();


    // copy information from the parent model.
    m_alpha = parent.alpha();
    m_mean = parent.m_mean;
    m_covariance = parent.m_covariance;

    m_creation_time = time;
    return;
}

/*! @brief Calculates the predicted measurement to the specified field location for the current estimated position.

@param xLocation field x coordinate of the location.
@param yLocation field y coordinate of the location.
*/
Matrix SelfModel::CalculateMeasurementPrediction(float xLocation, float yLocation) const
{
    return CalculateMeasurementPrediction(mean(), xLocation, yLocation);
}

/*! @brief Calculates the predicted measurement to the specified field location with an orientation for the current estimated position.

@param xLocation field x coordinate of the location.
@param yLocation field y coordinate of the location.
@param orientation field orientation of the object.
*/
Matrix SelfModel::CalculateMeasurementPrediction(float xLocation, float yLocation, float orientation) const
{
    return CalculateMeasurementPrediction(mean(), xLocation, yLocation, orientation);
}

/*! @brief Calculates the predicted measurement to the specified field location with an orientation.

@param state the state for which to calculate the prediction.
@param xLocation field x coordinate of the location.
@param yLocation field y coordinate of the location.
@param orientation field orientation of the object.
*/
Matrix SelfModel::CalculateMeasurementPrediction(const Matrix& state, float xLocation, float yLocation)
{
    float dX = xLocation - state[states_x][0];
    float dY = yLocation - state[states_y][0];
    Matrix result(2,1,false);
    result[0][0] = sqrt(dX*dX + dY*dY);
    result[1][0] = mathGeneral::normaliseAngle(atan2(dY,dX) - state[states_heading][0]);
    return result;
}

/*! @brief Calculates the predicted measurement to the specified field location with an orientation.

@param state the state for which to calculate the prediction.
@param xLocation field x coordinate of the location.
@param yLocation field y coordinate of the location.
@param orientation field orientation of the object.
*/
Matrix SelfModel::CalculateMeasurementPrediction(const Matrix& state, float xLocation, float yLocation, float orientation)
{
    //! TODO Implement orientation calculation.
    float dX = xLocation - state[states_x][0];
    float dY = yLocation - state[states_y][0];
    Matrix result(3,1,false);
    result[0][0] = sqrt(dX*dX + dY*dY);
    result[1][0] = mathGeneral::normaliseAngle(atan2(dY,dX) - state[states_heading][0]);
    return result;
}

/*! @brief Determine if the current position qualifies as lost.
    @return Ture if lost, false if not lost.
*/
bool SelfModel::isLost() const
{
    return false;
}

/*! @brief Generate the current self state from the current state estimate.
    @return Self object describing the current state estimate.
*/
Self SelfModel::GenerateSelfState() const
{
    Self result;
    result.updateLocationOfSelf(mean(states_x), mean(states_y), mean(states_heading), sd(states_x), sd(states_y), sd(states_heading),isLost());
    return result;
}

/*! @brief Get the historical parents of this model.
    @param steps_back The number of steps back along the decision tree to get the value for. This valaue should be in the range 1..history_depth.
    @return The model ID of the parent at the given step in the decision tree.
*/
unsigned int SelfModel::history(unsigned int steps_back) const
{
    unsigned int result = 0;
    unsigned int index = m_history_buffer.size() - steps_back;    // Oldest value is at [0], newest at [size - 1]
    if((index < m_history_buffer.size()) and (index >= 0))
    {
        result = m_history_buffer[index];
    }
    return result;
}

/*! @brief Get the previous decision path when this ambiguous object was last encountered.
    @param theObject The ambiguous object.
    @return The object id of the unique object that was last chosen.
*/
unsigned int SelfModel::previousSplitOption(const AmbiguousObject& theObject) const
{
    unsigned int result;
    unsigned int objectIndex = theObject.getID();
    assert(objectIndex < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
    result = m_previous_decisions[objectIndex];
    return result;
}


std::ostream& operator<< (std::ostream& output, const SelfModel& p_model)
{
    // Model member variables
    output.write(reinterpret_cast<const char*>(&p_model.m_active), sizeof(p_model.m_active));
    output.write(reinterpret_cast<const char*>(&p_model.m_alpha), sizeof(p_model.m_alpha));
    output.write(reinterpret_cast<const char*>(&p_model.m_id), sizeof(p_model.m_id));
    output.write(reinterpret_cast<const char*>(&p_model.m_parent_id), sizeof(p_model.m_parent_id));
    output.write(reinterpret_cast<const char*>(&p_model.m_split_option), sizeof(p_model.m_split_option));
    output.write(reinterpret_cast<const char*>(&p_model.m_creation_time), sizeof(p_model.m_creation_time));

    // Write values from base class.
    const Moment* moment = static_cast<const Moment*>(&p_model);
    output << (*moment);

    return output;
}

std::istream& operator>> (std::istream& input, SelfModel& p_model)
{
    // Model member variables
    input.read(reinterpret_cast<char*>(&p_model.m_active), sizeof(p_model.m_active));
    input.read(reinterpret_cast<char*>(&p_model.m_alpha), sizeof(p_model.m_alpha));
    input.read(reinterpret_cast<char*>(&p_model.m_id), sizeof(p_model.m_id));
    input.read(reinterpret_cast<char*>(&p_model.m_parent_id), sizeof(p_model.m_parent_id));
    input.read(reinterpret_cast<char*>(&p_model.m_split_option), sizeof(p_model.m_split_option));
    input.read(reinterpret_cast<char*>(&p_model.m_creation_time), sizeof(p_model.m_creation_time));

    // Read values to base class
    Moment* moment = static_cast<Moment*>(&p_model);
    input >> (*moment);

    return input;
}
