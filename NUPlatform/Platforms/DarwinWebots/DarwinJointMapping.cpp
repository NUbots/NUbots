#include "DarwinJointMapping.h"
#include "Tools/Math/General.h"


// From MX28.h file.
static const int CENTER_VALUE = 2048;
static const int MAX_VALUE = 4095;
static const double MIN_ANGLE = -180.0; // degree
static const double MAX_ANGLE = 180.0; // degree
static const double RATIO_VALUE2ANGLE = 0.088; // 360 / 4096
static const double RATIO_ANGLE2VALUE = 11.378; // 4096 / 360

/*! @brief Constructs a DarwinJointMapping class.

  The offset and muliplier tables are initialised.
 */
DarwinJointMapping::DarwinJointMapping()
{
    float temp_off[] =
    {
        //0.0f,
        //-0.523598667,
        -0.7854,         // [0]  HEAD_PITCH
        0.0f,           // [1]  HEAD_YAW
        -0.7853981f,    // [2]  L_SHOULDER_ROLL
        1.5707963f,     // [3]  L_SHOULDER_PITCH
        -1.5707963f,    // [4]  L_ELBOW
        0.7853981f,     // [5]  R_SHOULDER_ROLL
        -1.5707963f,    // [6]  R_SHOULDER_PITCH
        1.5707963,      // [7]  R_ELBOW
        0.0f,           // [8]  L_HIP_ROLL
        0.0f,           // [9]  L_HIP_PITCH
        0.0f,           // [10] L_HIP_YAW
        0.0f,           // [11] L_KNEE
        0.0f,           // [12] L_ANKLE_ROLL
        0.0f,           // [13] L_ANKLE_PITCH
        0.0f,           // [14] R_HIP_ROLL
        0.0f,           // [15] R_HIP_PITCH
        0.0f,           // [16] R_HIP_YAW
        0.0f,           // [17] R_KNEE
        0.0f,           // [18] R_ANKLE_ROLL
        0.0f,           // [19] R_ANKLE_PITCH
    };

    char temp_mult[] =
    {
        -1,             // [0]  HEAD_PITCH
        1,              // [1]  HEAD_YAW
        -1,             // [2]  L_SHOULDER_ROLL
        1,              // [3]  L_SHOULDER_PITCH
        1,              // [4]  L_ELBOW
        -1,             // [5]  R_SHOULDER_ROLL
        -1,             // [6]  R_SHOULDER_PITCH
        -1,             // [7]  R_ELBOW
        -1,             // [8]  L_HIP_ROLL
        -1,             // [9]  L_HIP_PITCH
        -1,             // [10] L_HIP_YAW
        -1,             // [11] L_KNEE
        1,              // [12] L_ANKLE_ROLL
        1,              // [13] L_ANKLE_PITCH
        -1,             // [14] R_HIP_ROLL
        1,              // [15] R_HIP_PITCH
        -1,             // [16] R_HIP_YAW
        1,              // [17] R_KNEE
        1,              // [18] R_ANKLE_ROLL
        -1,             // [19] R_ANKLE_PITCH
    };

    m_offsets = std::vector<float>(temp_off, temp_off + sizeof(temp_off)/sizeof(*temp_off));
    m_multipliers = std::vector<char>(temp_mult, temp_mult + sizeof(temp_mult)/sizeof(*temp_mult));
    return;
}

/*! @brief Converts a motor angle in radians into a Robotis MX28 motor value.

  @param radian The motor angle in radians.
  @return The Robotis MX28 motor value corresponding to the radian value parameter.
 */
int DarwinJointMapping::Radian2Value(float radian)
{
    static const float RATIO_RADIAN2VALUE = MAX_VALUE / (2*mathGeneral::PI);
    int value = (int)(radian*RATIO_RADIAN2VALUE)+CENTER_VALUE;
    return value;
}

/*! @brief Converts a Robotis MX28 motor value into a motor angle in radians.

  @param value The Robotis MX28 motor value.
  @return The motor angle in radians corresponding to the motor value parameter.
 */
float DarwinJointMapping::Value2Radian(int value)
{
    static const float RATIO_VALUE2RADIAN = (2*mathGeneral::PI) / MAX_VALUE;
    float radian = (float)(value-CENTER_VALUE)*RATIO_VALUE2RADIAN;
    return radian;
}

/*! @brief  Converts a joint angle in radians within the NUbots joint space into a Robotis MX28 motor value.

  The NUbots joint space is the standard joint angle definitions constant between all robots. In order to
  convert the angles into this space an offset and multiplier are applied to each angle.

  @param id The motor ID within the local joint list.
  @param value The Robotis MX28 motor value.
  @return The motor value with the offset and multipler values removed, corresponding to the joint angle parameter.
 */
int DarwinJointMapping::joint2raw(unsigned int id, float joint) const
{
    int raw = Radian2Value(m_multipliers[id] * joint - m_offsets[id]);
    return raw;
}

/*! @brief  Converts a Robotis MX28 motor value into a joint angle in radians within the NUbots joint space.

  The NUbots joint space is the standard joint angle definitions constant between all robots. In order to
  convert the angles into this space an offset and multiplier are applied to each angle.

  @param id The motor ID within the local joint list.
  @param value The raw Robotis MX28 motor value.
  @return The joint angle with the offset and multipler values applied, corresponding to the motor value parameter.
 */
float DarwinJointMapping::raw2joint(unsigned int id, int raw) const
{
    float joint = (Value2Radian(raw) + m_offsets[id]) / m_multipliers[id];
    return joint;
}

float DarwinJointMapping::rad2joint(unsigned int id, float radian) const
{
    float joint = (radian + m_offsets[id]) / m_multipliers[id];
    return joint;
}

float DarwinJointMapping::joint2rad(unsigned int id, float joint) const
{
    float rad = m_multipliers[id] * joint - m_offsets[id];
    return rad;
}

/*! @brief  Converts a vector of joint angles in radians within the NUbots joint space into a vecor of raw Robotis MX28 motor values.

  The NUbots joint space is the standard joint angle definitions constant between all robots. In order to
  convert the angles into this space an offset and multiplier are applied to each angle.

  @param body The vector of joint angle to be converted.
  @return The vector of raw motor values with the offset and multipler values removed, corresponding to the joint angles given as a parameter.
 */
std::vector<int> DarwinJointMapping::body2raw(const std::vector<float>& body) const
{
    std::vector<int> raw(body.size(), 0);
    for(unsigned int id = 0; id < body.size(); ++id)
    {
        raw[id] = joint2raw(id, body[id]);
    }
    return raw;
}

/*! @brief  Converts a vector of Robotis MX28 motor values into a vecor of joint angles in radians within the NUbots joint space.

  @param raw The raw Robotis MX28 motor value.
  @return The vector of joint angles with the offset and multipler values applied, corresponding to the motor values given as a parameter.
 */
std::vector<float> DarwinJointMapping::raw2body(const std::vector<int>& raw) const
{
    std::vector<float> body(raw.size(), 0);
    for(unsigned int id = 0; id < raw.size(); ++id)
    {
        body[id] = raw2joint(id, raw[id]);
    }
    return body;
}
