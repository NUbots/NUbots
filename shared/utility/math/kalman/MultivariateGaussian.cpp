#include "MultivariateGaussian.h"
#include "Utils.h"
#include <sstream>
#include <iostream>
#include <assert.h>
//#include "Utils.h" //utility class needed for matrix comparison
//#include <armadillo> //already included in MultivariateGaussian.h

//bool isMatValid(const arma::mat& m) { //used in kalman.cpp as well, moved to Utils.h file
//    if(m.n_rows == 0 or m.n_cols == 0)
//        return false;
//    return true;
//}


MultivariateGaussian::MultivariateGaussian(): m_numStates(0) { // Creates a new moment with zero states. This is a null representation.
    m_mean = arma::mat(m_numStates,1);
    m_covariance = arma::mat(m_numStates, m_numStates);
}

MultivariateGaussian::MultivariateGaussian(unsigned int numStates): m_numStates(numStates) { // @brief Size specified constructor creates a new moment desribing the number of states specified. @param numStates the number fo states described by the moment.
    m_mean = arma::mat(m_numStates,1);
    m_covariance = arma::mat(m_numStates, m_numStates);
}

MultivariateGaussian::MultivariateGaussian(const MultivariateGaussian& source): m_numStates(source.m_numStates) {
    m_mean = source.m_mean;
    m_covariance = source.m_covariance;
}

MultivariateGaussian::MultivariateGaussian(const arma::mat& mean, const arma::mat& covariance) {
    const unsigned int size = mean.n_rows;
    assert(size == covariance.n_cols);
    assert(size == covariance.n_rows);

    m_numStates = size;
    setMean(mean);
    setCovariance(covariance);
    return;
}

MultivariateGaussian::~MultivariateGaussian() {} //Empty destructor

MultivariateGaussian& MultivariateGaussian::operator=(const MultivariateGaussian& source) {
    if (this != &source) { // protect against invalid self-assignment
        m_numStates = source.m_numStates;
        setMean(source.mean());
        setCovariance(source.covariance());
    }
    // by convention, always return *this
    return *this;
}

//*
bool MultivariateGaussian::operator ==(const MultivariateGaussian& b) const { 
    if( m_numStates != b.m_numStates) {
        return false;
    }//FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if(arma::any(arma::vectorise(m_mean!=b.m_mean))) { // because armadillo doesnt have a simple A!=B matrix comparison function //m_mean != b.m_mean
        return false;
    }
    if(arma::any(arma::vectorise(this->m_covariance!=b.m_covariance))) {
        return false;
    }
    
    return true;
}//*/

float MultivariateGaussian::mean(unsigned int stateNumber) const { // @brief Returns the mean of the specified state. @param stateNumber the number of the desired state.
    if(stateNumber < m_numStates)
        return m_mean(stateNumber, 0);
    return 0.0f;
}

arma::mat MultivariateGaussian::mean() const { // @brief Returns the mean of the moment.
    return m_mean;
}


arma::mat MultivariateGaussian::covariance() const { // @brief Returns the covariance of the moment
    return m_covariance;
}

float MultivariateGaussian::covariance(unsigned int row, unsigned int col) const {
    if( (row < m_numStates) and (col < m_numStates) )
        return m_covariance(row, col);
    return 0.0f;
}

float MultivariateGaussian::sd(unsigned int stateNumber) const { // @brief Returns the standard deviation of the specified state. @param stateNumber the number of the desired state.
    return sqrt(variance(stateNumber));
}

float MultivariateGaussian::variance(unsigned int stateNumber) const { // @brief Returns the variance of the specified state. @param stateNumber the number of the desired state.
    if(stateNumber < m_numStates)
        return m_covariance(stateNumber, stateNumber);
    return 0.0f;
}


void MultivariateGaussian::setMean(const arma::mat& newMean) { // @brief Sets the mean of the moment to the given value/s. The dimensions of the new mean matrix must match the number of states for moment. @param newMean the new value for the mean.
    bool isCorrectSize = ((unsigned int)newMean.n_rows == m_numStates) && (newMean.n_cols == 1);
    assert(isCorrectSize);
    assert(duti::isMatrixValid(newMean));
    if(isCorrectSize and duti::isMatrixValid(newMean)) {
        m_mean = newMean;
    }
    return;
}

void MultivariateGaussian::setCovariance(const arma::mat& newCovariance) { // @brief Sets the covariance of the moment to the given value/s. The dimensions of the new covariance matrix must match the number of states for moment. @param newCovariance the new value for the covariance.
    bool isCorrectSize = (newCovariance.n_rows == m_numStates) && (newCovariance.n_cols == m_numStates);
    assert(isCorrectSize);
    assert(duti::isMatrixValid(newCovariance));
    if(isCorrectSize and duti::isMatrixValid(newCovariance)) {
        m_covariance = newCovariance;
    }
    return;
}

bool MultivariateGaussian::isNull() const { // @brief Determines if the moment is null. this is the case if it contains zero states.
    return (m_numStates < 1);
}

std::string MultivariateGaussian::string() const { // @brief Return the current moment as a human-readable string for display.
    std::stringstream result;
    result << "Mean: " << mean().t(); //output the transpose of the mean
    result << "Covariance: " << std::endl;
    result << covariance();
    return result.str();
}

template <class T>
const char * c_cast(const T& input) {
    return reinterpret_cast<const char*>(&input);
}

void MultivariateGaussian::writeData(std::ostream& output) const {
    char header[] = {"m"};
    output.write(header,1);
    float temp;
    for(unsigned int i = 0; i < m_numStates; i++) {
        temp = mean(i);
        output.write(c_cast(temp), sizeof(temp));
    }
    for (int r = 0; r < m_covariance.n_rows; r++) {
        for (int c = 0; c < m_covariance.n_cols; c++) {
            temp = m_covariance(r, c);
            output.write(c_cast(temp), sizeof(temp));
        }
    }
    return;
}

/* 
std::ostream& MultivariateGaussian::writeStreamBinary (std::ostream& output) const {
    output.write(reinterpret_cast<const char*>(&m_numStates), sizeof(m_numStates));
    //WriteMatrix(output, m_mean);       //ox---------------------------------------------------- THIS FUNCTION IS NOT PORTED FULLY, DONT INTEND TO USE
    //WriteMatrix(output, m_covariance); //ox----------------------------------------------------
    return output;
}

std::istream& MultivariateGaussian::readStreamBinary (std::istream& input) {
    input.read(reinterpret_cast<char*>(&m_numStates), sizeof(m_numStates));
    //m_mean = ReadMatrix(input);       //ox---------------------------------------------------- THIS FUNCTION IS NOT PORTED FULLY, DONT INTEND TO USE
    //m_covariance = ReadMatrix(input); //ox----------------------------------------------------
    return input;
}


std::ostream& operator<< (std::ostream& output, const MultivariateGaussian& p_moment) {
    return p_moment.writeStreamBinary(output);
}

std::istream& operator>> (std::istream& input, MultivariateGaussian& p_moment) {
    return p_moment.readStreamBinary(input);
}
*/