#include "SelfModel.h"
#include "Tools/Math/General.h"

/*! @brief Default time constructor

    This constructor requires a creation time.

 */
SelfModel::SelfModel(float time): Moment(states_total)
{
    m_id = GenerateId();
    m_creation_time = time;
    return;
}

/*! @brief Copy constructor

    Creates an exact copy of the original model, Preserving id, parent and splitting information.
 */
SelfModel::SelfModel(const SelfModel& source): Moment(states_total)
{
    m_active = source.m_active;
    m_alpha =  source.m_alpha;
    m_covariance = source.m_covariance;
    m_id = source.m_id;
    m_mean = source.m_mean;
    m_parent_id = source.m_parent_id;
    m_split_option = source.m_split_option;
    m_creation_time = source.m_creation_time;
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
    m_split_option = splitOption.getID();

    // copy information from the parent model.
    m_alpha = parent.alpha();
    m_mean = parent.m_mean;
    m_covariance = parent.m_covariance;

    m_creation_time = time;
    // Perform the ambiguous object update
    //! TODO Missing ambiguous object update.

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
