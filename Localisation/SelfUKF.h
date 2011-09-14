#ifndef SELFUKF_H
#define SELFUKF_H
#include "SelfModel.h"
#include "Infrastructure/FieldObjects/StationaryObject.h"
#include "odometryMotionModel.h"

class MeasurementError
{
public:
    float distance() const {return m_distance;}
    float heading() const {return m_heading;}
    void setDistance(float newDistance) {m_distance = newDistance;}
    void setHeading(float newHeading) {m_heading = newHeading;}
private:
    float m_distance;
    float m_heading;
};

class SelfUKF: public SelfModel
{
public:
    enum updateResult
    {
        RESULT_OK,
        RESULT_OUTLIER,
    };
    // Constructors
    SelfUKF();
    SelfUKF(double time);
    SelfUKF(const SelfUKF& source);
    SelfUKF(const SelfUKF& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time);

    void InitialiseCachedValues();

    // Update functions
    updateResult TimeUpdate(const std::vector<float>& odometry, OdometryMotionModel& motion_model, float deltaTime);
    updateResult MultipleObjectUpdate(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement);

    updateResult MeasurementUpdate(const std::vector<StationaryObject*>& objects);
    updateResult MeasurementUpdate(const StationaryObject& object, const MeasurementError& error);

    Matrix CalculateSigmaPoints() const;
    float CalculateAlphaWeighting(const Matrix& innovation, const Matrix& innovationVariance, float outlierLikelyhood) const;

    bool clipState(int stateIndex, double minValue, double maxValue);

    /*!
    @brief Output streaming operation.
    @param output The output stream.
    @param p_kf The source kalman filter to be streamed.
    */
    friend std::ostream& operator<< (std::ostream& output, const SelfUKF& p_model);

    /*!
    @brief Input streaming operation.
    @param input The input stream.
    @param p_kf The destination kalman filter to be streamed to.
    */
    friend std::istream& operator>> (std::istream& input, SelfUKF& p_model);

    std::string summary(bool brief=true) const;


protected:
    Matrix sqrtOfTestWeightings; // Square root of W (Constant)
    Matrix sqrtOfProcessNoise; // Square root of Process Noise (Q matrix). (Constant)
    static const float c_Kappa;

};



#endif // SELFUKF_H
