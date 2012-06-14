#ifndef MOMENT_H
#define MOMENT_H

#include "Matrix.h"
#include <string>

/*!
  * A class used to represent the second moment (mean and covariance) of n states.
  */

class Moment
{
public:
    Moment();
    Moment(unsigned int numStates);
    Moment(const Moment& source);
    virtual ~Moment()
    {
    }

    float mean(unsigned int stateNumber) const;
    Matrix mean() const;
    float sd(unsigned int stateNumber) const;
    Matrix covariance() const;
    float covariance(unsigned int row, unsigned int col) const;
    float variance(unsigned int stateNumber) const;
    virtual void setMean(const Matrix& newMean);
    virtual void setCovariance(const Matrix& newCovariance);
    bool isNull() const;
    std::string string() const;
    void writeData(std::ostream& output) const;
    unsigned int totalStates() const  {return m_numStates;}

    Moment& operator= (const Moment & source);
    bool operator ==(const Moment& b) const;
    bool operator !=(const Moment& b) const
    {return (!((*this) == b));}

    /*!
    @brief Outputs a binary representation of the Moment object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    std::ostream& writeStreamBinary (std::ostream& output) const;

    /*!
    @brief Reads in a Moment from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    std::istream& readStreamBinary (std::istream& input);

    /*!
    @brief Output streaming operation.
    @param output The output stream.
    @param p_moment The source moment to be streamed.
    */
    friend std::ostream& operator<< (std::ostream& output, const Moment& p_moment);

    /*!
    @brief Input streaming operation.
    @param input The input stream.
    @param p_moment The destination moment to be streamed to.
    */
    friend std::istream& operator>> (std::istream& input, Moment& p_moment);

protected:
    unsigned int m_numStates;
    Matrix m_mean;
    Matrix m_covariance;
};

#endif // MOMENT_H
