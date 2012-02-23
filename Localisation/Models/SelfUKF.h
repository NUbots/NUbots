#ifndef SELFUKF_H
#define SELFUKF_H
#include "SelfModel.h"
#include "Infrastructure/FieldObjects/StationaryObject.h"
#include "Localisation/odometryMotionModel.h"
#include "Localisation/MeasurementError.h"

class SelfUKF: public SelfModel
{
public:
    // Constructors
    SelfUKF();
    SelfUKF(double time);
    SelfUKF(const SelfModel& source);
    SelfUKF(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time);

    void InitialiseCachedValues();
    Matrix CalculateWeights(unsigned int num_states, bool covariance);
    // Update functions
    updateResult TimeUpdate(const std::vector<float>& odometry, OdometryMotionModel& motion_model, float deltaTime);
    updateResult MeasurementUpdate(const StationaryObject& object, const MeasurementError& error);
    updateResult MultipleObjectUpdate(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement);
    updateResult MeasurementUpdate(const AmbiguousObject& object, const std::vector<StationaryObject*>& possible_objects, const MeasurementError& error);



    Matrix CalculateSigmaPoints(Matrix mean, Matrix covariance) const;
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

protected:
    Matrix m_process_noise; // Square root of Process Noise (Q matrix). (Constant)
    Matrix m_mean_weights;
    Matrix m_covariance_weights;
    static const float c_Kappa;
    double m_alpha_2;
    double m_beta;
    double m_x;

};



#endif // SELFUKF_H
