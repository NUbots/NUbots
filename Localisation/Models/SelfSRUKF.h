#ifndef SELFSRUKF_H
#define SELFSRUKF_H
#include "SelfModel.h"
#include "Infrastructure/FieldObjects/StationaryObject.h"
#include "Localisation/odometryMotionModel.h"
#include "Localisation/MeasurementError.h"

class SelfSRUKF: public SelfModel
{
public:
    // Constructors
    SelfSRUKF();
    SelfSRUKF(double time);
    SelfSRUKF(const SelfModel& source);
    SelfSRUKF(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time);
    virtual ~SelfSRUKF()
    {
    }

    void InitialiseCachedValues();

    // Update functions
    updateResult TimeUpdate(const std::vector<float>& odometry, OdometryMotionModel& motion_model, float deltaTime);
    updateResult MultipleObjectUpdate(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement);
    updateResult MeasurementUpdate(const StationaryObject& object, const MeasurementError& error);
    updateResult MeasurementUpdate(const AmbiguousObject& object, const std::vector<StationaryObject*>& possible_objects, const MeasurementError& error);


    void setCovariance(const Matrix& newCovariance);
    void setSqrtCovariance(const Matrix& newSqrtCovariance);


    Matrix CalculateSigmaPoints() const;
    float CalculateAlphaWeighting(const Matrix& innovation, const Matrix& innovationVariance, float outlierLikelyhood) const;

    bool operator ==(const SelfSRUKF& b) const;
    bool operator !=(const SelfSRUKF& b) const
    {return (!((*this) == b));}

    /*!
    @brief Outputs a binary representation of the UKF object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    std::ostream& writeStreamBinary (std::ostream& output) const;

    /*!
    @brief Reads in a UKF object from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    std::istream& readStreamBinary (std::istream& input);

    /*!
    @brief Output streaming operation.
    @param output The output stream.
    @param p_kf The source kalman filter to be streamed.
    */
    friend std::ostream& operator<< (std::ostream& output, const SelfSRUKF& p_model);

    /*!
    @brief Input streaming operation.
    @param input The input stream.
    @param p_kf The destination kalman filter to be streamed to.
    */
    friend std::istream& operator>> (std::istream& input, SelfSRUKF& p_model);


protected:
    Matrix sqrtOfTestWeightings; // Square root of W (Constant)
    Matrix sqrtOfProcessNoise; // Square root of Process Noise (Q matrix). (Constant)
    static const float c_Kappa;
    Matrix m_sqrt_covariance;

};



#endif // SELFUKF_H
