 /*this was the old interface (IKFmodel.h) that IMUModel used to inherit from
  We got rid of the interface and instead replaced it with template metaprogramming for efficiency (albiet a bit more complex)
  essentially, EVERY KALMAN FILTER MODEL SHOULD CONTAIN THE FOLLOWING 2 FUNCTIONS:
  
        * processequation()     used to update the current prediction during the time update
        * measurementequation() used to calculate the predicted measurement from the estimated state of the system

Copyright (c) 2012 Steven Nicklin, INTERFACE: IKFModel.h

class IKFModel{ //-------------------------the original interface IMUModel extends from
public:
    virtual IKFModel* Clone() = 0;
    virtual ~IKFModel() {}
    virtual Matrix processEquation(const Matrix& state, double deltaT, const Matrix& measurement) = 0;
    virtual Matrix measurementEquation(const Matrix& state, const Matrix& measurementArgs, unsigned int type) = 0;
    virtual Matrix measurementDistance(const Matrix& measurement1, const Matrix& measurement2, unsigned int type) = 0;
    virtual void limitState(Matrix& state) = 0;
    virtual unsigned int totalStates() const = 0;
    virtual std::ostream& writeStreamBinary (std::ostream& output) const = 0;
    virtual std::istream& readStreamBinary (std::istream& input) = 0;
};
*/
#include <armadillo>
#ifndef IMUModel_H
#define IMUModel_H

class IMUModel {
public:
    //void test() { //--------------------------TEMPORARY TEST ~ DELETE ME
    //    arma::mat teststate(9, 9); teststate.ones();
    //    arma::mat testargs(9, 9);  testargs.ones();    
    //  accelerometerMeasurementEquation(teststate, testargs); //--------------------------
    //} //--------------------------
    
    enum State {
        kstates_gyro_offset_x,
        kstates_gyro_offset_y,
        kstates_body_angle_x,
        kstates_body_angle_y,
        kstates_total
    };

    enum MeasurementType {
        kmeasurement_accelerometer,
        kmeasurement_kinematic,
        kmeasurement_total_types
    };

    IMUModel(); // empty constructor
    IMUModel* Clone() { return new IMUModel(*this); } //this should Ideally be removed and replaced with a smart-pointer equivilant

    arma::mat processEquation(const arma::mat& state, double deltaT, const arma::mat& measurement);      // The process equation, this describes the transition of the estimate due to time and inputs applied. @param state The state determined frim the previous estimate. @param deltaT The elapsed time since the previous update was performed. @param measurement Measurment data obtained from the inputs to the system. @return The new updated measurement.
    arma::mat measurementEquation(const arma::mat& state, const arma::mat& measurementArgs, unsigned int type);    // The measurement equation, this is used to calculate the expected measurement given a state of the system. @param state The estimated state of the system. @param measurementArgs Additional information about the measurement. @return The expected measurment for the given conditions.
    arma::mat measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2, unsigned int type);

    void limitState(arma::mat &state);
    unsigned int totalStates() const {
        return kstates_total;
    }

    std::ostream& writeStreamBinary (std::ostream& output) const; // @brief Outputs a binary representation of the UKF object to a stream. @param output The output stream. @return The output stream.
    std::istream& readStreamBinary (std::istream& input); // @brief Reads in a UKF object from the input stream. @param input The input stream. @return The input stream.

protected:
    IMUModel(const IMUModel& source);
    arma::mat kinematicMeasurementEquation(const arma::mat& state, const arma::mat& measurementArgs);
    arma::mat accelerometerMeasurementEquation(const arma::mat& state, const arma::mat& measurementArgs);
};

#endif