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
    float mean(unsigned int stateNumber) const;
    Matrix mean() const;
    float sd(unsigned int stateNumber) const;
    Matrix covariance() const;
    float variance(unsigned int stateNumber) const;
    void setMean(const Matrix& newMean);
    void setCovariance(const Matrix& newCovariance);
    bool isNull() const;
    std::string string() const;
    void writeData(std::ostream& output) const;
protected:
    const unsigned int m_numStates;
    Matrix m_mean;
    Matrix m_covariance;
};

#endif // MOMENT_H
