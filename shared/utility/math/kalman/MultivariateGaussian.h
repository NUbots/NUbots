/* 
 * File:   MultivariateGaussian.h
 * Author: Andrew
 *
 * Created on 4 December 2013, 8:54 PM
 * a class used to respresent the 2nd moment (mean & covariance) of n states
 * ported from old system: robocup/tools/math/multivariategaussian
 */

#ifndef MULTIVARIATEGAUSSIAN_H
#define	MULTIVARIATEGAUSSIAN_H
#include <armadillo>
#include <string>

class MultivariateGaussian {
//----------------------------------------------
public:
    MultivariateGaussian();
    MultivariateGaussian(unsigned int numStates);
    MultivariateGaussian(const MultivariateGaussian& source);
    MultivariateGaussian(const arma::mat& mean, const arma::mat& covariance);
    virtual ~MultivariateGaussian() {}

    arma::mat mean() const;
    arma::mat covariance() const;
    
    float mean(unsigned int stateNumber) const;
    float sd(unsigned int stateNumber) const;
    float covariance(unsigned int row, unsigned int col) const;
    float variance(unsigned int stateNumber) const;
    virtual void setMean(const arma::mat& newMean);
    virtual void setCovariance(const arma::mat& newCovariance);
    bool isNull() const;
    std::string string() const;
    void writeData(std::ostream& output) const;
    unsigned int totalStates() const  {return m_numStates;}

    MultivariateGaussian& operator= (const MultivariateGaussian & source);

    //!EXCLUDED FROM PORT----------------------------------------------------
    //std::ostream& writeStreamBinary (std::ostream& output) const; //@brief Outputs a binary representation of the MultivariateGaussian object to a stream. @param output The output stream. @return The output stream.
    //std::istream& readStreamBinary (std::istream& input); // @brief Reads in a MultivariateGaussian from the input stream. @param input The input stream. @return The input stream.

    bool operator !=(const MultivariateGaussian& b) const {return (!((*this) == b));}
    bool operator ==(const MultivariateGaussian& b) const;
    friend std::ostream& operator<< (std::ostream& output, const MultivariateGaussian& p_moment); // @brief Output streaming operation. @param output The output stream. @param p_moment The source moment to be streamed.
    friend std::istream& operator>> (std::istream& input, MultivariateGaussian& p_moment);    // @brief Input streaming operation. @param input The input stream. @param p_moment The destination moment to be streamed to.

protected:
    unsigned int m_numStates;
    arma::mat m_mean;
    arma::mat m_covariance;
//---------------------------------------------
};

#endif	/* MULTIVARIATEGAUSSIAN_H */

