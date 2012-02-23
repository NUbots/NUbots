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
    float mean(unsigned int stateNumber) const;
    Matrix mean() const;
    float sd(unsigned int stateNumber) const;
    Matrix covariance() const;
    float covariance(unsigned int row, unsigned int col) const;
    float variance(unsigned int stateNumber) const;
    void setMean(const Matrix& newMean);
    virtual void setCovariance(const Matrix& newCovariance);
    bool isNull() const;
    std::string string() const;
    void writeData(std::ostream& output) const;

    Moment& operator= (const Moment & source);


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
