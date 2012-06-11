#include "SelfModel.h"
#include "Tools/Math/General.h"
#include <sstream>

/*! @brief Default time constructor

    This constructor requires a creation time.

 */
SelfModel::SelfModel(float time): Moment(states_total), WeightedModel(time)
{
    m_previous_decisions.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, FieldObjects::NUM_STAT_FIELD_OBJECTS);
    return;
}

/*! @brief Copy constructor

    Creates an exact copy of the original model, Preserving id, parent and splitting information.
 */
SelfModel::SelfModel(const SelfModel& source): Moment(source), WeightedModel(source)
{
    m_split_option = source.m_split_option;
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
SelfModel::SelfModel(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, float time): Moment(parent), WeightedModel(parent,time)
{
    // Save the information about the model
    m_split_option = splitOption.getID();
    assert(object.getID() < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
    m_previous_decisions = parent.m_previous_decisions;
    m_previous_decisions[object.getID()] = splitOption.getID();
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

bool SelfModel::clipState(int stateIndex, double minValue, double maxValue){
    bool clipped = false;
    if(m_mean[stateIndex][0] > maxValue){
        double mult, Pii;
        Matrix Si;
        Si = m_covariance.getRow(stateIndex);
        Pii = convDble(Si * Si.transp());
        mult = (m_mean[stateIndex][0] - maxValue) / Pii;
        m_mean = m_mean - mult * m_covariance * Si.transp();
        m_mean[stateIndex][0] = maxValue;
        clipped = true;
    }
    if(m_mean[stateIndex][0] < minValue){
        double mult, Pii;
        Matrix Si;
        Si = m_covariance.getRow(stateIndex);
        Pii = convDble(Si * Si.transp());
        mult = (m_mean[stateIndex][0] - minValue) / Pii;
        m_mean = m_mean - mult * m_covariance * Si.transp();
        m_mean[stateIndex][0] = minValue;
        clipped = true;
    }
    m_mean[states_heading][0] = mathGeneral::normaliseAngle(m_mean[states_heading][0]);
    return clipped;
}

std::ostream& SelfModel::writeStreamBinary (std::ostream& output) const
{
    WeightedModel::writeStreamBinary(output);
    Moment::writeStreamBinary(output);
    output.write(reinterpret_cast<const char*>(&m_split_option), sizeof(m_split_option));
    return output;
}

std::istream& SelfModel::readStreamBinary (std::istream& input)
{
    WeightedModel::readStreamBinary(input);
    Moment::readStreamBinary(input);
    input.read(reinterpret_cast<char*>(&m_split_option), sizeof(m_split_option));
    return input;
}

std::ostream& operator<< (std::ostream& output, const SelfModel& p_model)
{
    return p_model.writeStreamBinary(output);
}

std::istream& operator>> (std::istream& input, SelfModel& p_model)
{
    return p_model.readStreamBinary(input);
}

std::string SelfModel::summary(bool brief) const
{
    std::stringstream buffer;
    buffer << WeightedModel::summary(brief);
    buffer << "Position: (" << mean(states_x) << "," << mean(states_y) << "," << mean(states_heading) << ")" << std::endl;
    if(!brief)
    {
            buffer << "Covariance - Position:" << std::endl << m_covariance << std::endl;
    }
    return buffer.str();
}

