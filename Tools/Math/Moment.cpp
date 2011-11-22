#include "Moment.h"
#include <sstream>

/*! @brief Default constructor

     Creates a new moment with zero states. This is a null representation.

 */
Moment::Moment(): m_numStates(0)
{
    m_mean = Matrix(m_numStates,0,false);
    m_covariance = Matrix(m_numStates, m_numStates, false);
}

/*! @brief Size specified constructor

     Creates a new moment desribing the number of states specified.

    @param numStates the number fo states described by the moment.
 */
Moment::Moment(unsigned int numStates): m_numStates(numStates)
{
    m_mean = Matrix(m_numStates,0,false);
    m_covariance = Matrix(m_numStates, m_numStates, false);
}

/*! @brief Returns the mean of the specified state.

    @param stateNumber the number of the desired state.
 */
float Moment::mean(unsigned int stateNumber) const
{
    if(stateNumber < m_numStates)
        return m_mean[stateNumber][0];
    return 0.0f;
}

/*! @brief Returns the mean of the moment.
 */
Matrix Moment::mean() const
{
    return m_mean;
}

/*! @brief Returns the covariance of the moment
 */
Matrix Moment::covariance() const
{
    return m_covariance;
}

/*! @brief Returns the standard deviation of the specified state.

    @param stateNumber the number of the desired state.
 */
float Moment::sd(unsigned int stateNumber) const
{
    return sqrt(variance(stateNumber));
}

/*! @brief Returns the variance of the specified state.

    @param stateNumber the number of the desired state.
 */
float Moment::variance(unsigned int stateNumber) const
{
    if(stateNumber < m_numStates)
        return m_covariance[stateNumber][stateNumber];
    return 0.0f;
}

/*! @brief Sets the mean of the moment to the given value/s.

    The dimensions of the new mean matrix must match the number of states for moment.

    @param newMean the new value for the mean.
 */
void Moment::setMean(const Matrix& newMean)
{
    if( ((unsigned int)newMean.getm() == m_numStates) && newMean.getn() == 1)
    {
        m_mean = newMean;
    }
    return;
}

/*! @brief Sets the covariance of the moment to the given value/s.

    The dimensions of the new covariance matrix must match the number of states for moment.

    @param newCovariance the new value for the covariance.
 */
void Moment::setCovariance(const Matrix& newCovariance)
{
    if( ((unsigned int)newCovariance.getm() == m_numStates) && ((unsigned int)newCovariance.getn() == m_numStates))
    {
        m_covariance = newCovariance;
    }
    return;
}

/*! @brief Determines if the moment is null. this is the case if it contains zero states.
 */
bool Moment::isNull() const
{
    return (m_numStates < 1);
}

/*! @brief Return the current moment as a human-readable string for display.
 */
std::string Moment::string() const
{
    std::stringstream result;
    result << "Mean: " << mean() << std::endl;
    result << "Covariance: " << std::endl;
    result << covariance();
    return result.str();
}

template <class T>
const char * c_cast(const T& input)
{
    return reinterpret_cast<const char*>(&input);
}

void Moment::writeData(std::ostream& output) const
{
    char header[] = {"m"};
    output.write(header,1);
    float temp;
    for(unsigned int i = 0; i < m_numStates; i++)
    {
        temp = mean(i);
        output.write(c_cast(temp), sizeof(temp));
    }
    for (int r = 0; r < m_covariance.getm(); r++)
    {
        for (int c = 0; c < m_covariance.getn(); c++)
        {
            temp = m_covariance[r][c];
            output.write(c_cast(temp), sizeof(temp));
        }
    }
    return;
}
