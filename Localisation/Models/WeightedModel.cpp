#include "WeightedModel.h"
#include <sstream>

/*! @brief Default time constructor

    This constructor requires a creation time.

 */
WeightedModel::WeightedModel(float time)
{
    const unsigned int history_depth = 5;
    m_id = GenerateId();    // calculate unique id.
    m_creation_time = time;
    m_alpha = 1.0f;
    m_parent_history_buffer = boost::circular_buffer<unsigned int>(history_depth);
    return;
}

/*! @brief Copy constructor

    Creates an exact copy of the original model, Preserving id, parent and splitting information.
 */
WeightedModel::WeightedModel(const WeightedModel& source)
{
    m_active = source.m_active;
    m_alpha =  source.m_alpha;
    m_id = source.m_id;
    m_parent_id = source.m_parent_id;
    m_creation_time = source.m_creation_time;
    m_parent_history_buffer = source.m_parent_history_buffer;
    return;
}

/*! @brief Child constructor.
    Creates a child model from the given parent.
    @param parent The parent model.
    @param time Creation time (current time).
 */
WeightedModel::WeightedModel(const WeightedModel& parent, float time)
{
    m_active = false;
    m_alpha = parent.alpha();
    m_id = GenerateId();
    m_parent_id = parent.id();
    m_creation_time = time;
    m_parent_history_buffer = parent.m_parent_history_buffer; // Get parents history.
    m_parent_history_buffer.push_back(m_parent_id);    // Add parent to the history.
}

/*! @brief Get the historical parents of this model.
    @param steps_back The number of steps back along the decision tree to get the value for. This value should be in the range 1..history_depth.
    @return The model ID of the parent at the given step in the decision tree.
*/
unsigned int WeightedModel::history(unsigned int steps_back) const
{
    unsigned int result = 0;
    unsigned int index = m_parent_history_buffer.size() - steps_back;    // Oldest value is at [0], newest at [size - 1]
    if((index < m_parent_history_buffer.size()) and (index >= 0))
    {
        result = m_parent_history_buffer.at(index);
    }
    return result;
}

/*!
@brief Outputs a binary representation of the UKF object to a stream.
@param output The output stream.
@return The output stream.
*/
std::ostream& WeightedModel::writeStreamBinary (std::ostream& output) const
{
    // Model member variables
    output.write(reinterpret_cast<const char*>(&m_active), sizeof(m_active));
    output.write(reinterpret_cast<const char*>(&m_alpha), sizeof(m_alpha));
    output.write(reinterpret_cast<const char*>(&m_id), sizeof(m_id));
    output.write(reinterpret_cast<const char*>(&m_parent_id), sizeof(m_parent_id));
    output.write(reinterpret_cast<const char*>(&m_creation_time), sizeof(m_creation_time));

    // Write the parent history buffer.
    unsigned int buffer_size = m_parent_history_buffer.size(), value;
    output.write(reinterpret_cast<const char*>(&buffer_size), sizeof(buffer_size));
    for(unsigned int i = 0; i < buffer_size; ++i)
    {
        value = m_parent_history_buffer.at(i);
        output.write(reinterpret_cast<const char*>(&value), sizeof(value));
    }


    return output;
}

/*!
@brief Reads in a UKF object from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& WeightedModel::readStreamBinary (std::istream& input)
{
    // Model member variables
    input.read(reinterpret_cast<char*>(&m_active), sizeof(m_active));
    input.read(reinterpret_cast<char*>(&m_alpha), sizeof(m_alpha));
    input.read(reinterpret_cast<char*>(&m_id), sizeof(m_id));
    input.read(reinterpret_cast<char*>(&m_parent_id), sizeof(m_parent_id));
    input.read(reinterpret_cast<char*>(&m_creation_time), sizeof(m_creation_time));

    // Read the parent history buffer.
    unsigned int buffer_size, value;
    input.read(reinterpret_cast<char*>(&buffer_size), sizeof(buffer_size));
    m_parent_history_buffer.resize(buffer_size);
    for(unsigned int i = 0; i < buffer_size; ++i)
    {
        input.read(reinterpret_cast<char*>(&value), sizeof(value));
        m_parent_history_buffer.at(i) = value;
    }

    return input;
}

/*! @brief Generate a string with a description of the current weighted model.
    @param brief If true a brief summary is produce. If false a more detailed summary is created.
    @return String containing the summary.
*/
std::string WeightedModel::summary(bool brief) const
{
    std::stringstream buffer;
    buffer << "Model Id: " << id() << " Active: " << active() << " Alpha: " << alpha() << std::endl;
    if(!brief)
    {
            if(m_parent_history_buffer.size() > 0)
            {
                buffer << "History: ";

                for(boost::circular_buffer<unsigned int>::const_iterator buff_it = m_parent_history_buffer.begin(); buff_it != m_parent_history_buffer.end(); ++buff_it)
                {
                    if(buff_it != m_parent_history_buffer.begin())
                    {
                        buffer << "->";
                    }
                    buffer << (*buff_it);
                }
                buffer << std::endl;
            }
    }
    return buffer.str();
}
