#include "SelfUKF.h"
#include "Tools/Math/General.h"
#include <sstream>
#include <iostream>

// Define Constants
const float SelfUKF::c_Kappa = 1.0f;

/*! @brief Default constructor
 */
SelfUKF::SelfUKF(): SelfModel(0.0)
{
    InitialiseCachedValues();
}

/*! @brief Default time constructor

    This constructor requires a creation time.

 */
SelfUKF::SelfUKF(double time): SelfModel(time)
{
    InitialiseCachedValues();
}

/*! @brief Copy constructor

 */
SelfUKF::SelfUKF(const SelfModel& source): SelfModel(source)
{
    InitialiseCachedValues();
}

/*! @brief Split constructor

Takes a parent filter and performs a split on an ambiguous object using the given split option.

@param parent The parent model from which the split orignates.
@param object The ambiguous object belong evaluated by the split.
@param splitOption The option to be evaluated within this model.
@param time The current time of the update
*/
SelfUKF::SelfUKF(const SelfModel& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time):
        SelfModel(parent, object, splitOption, time)
{
    InitialiseCachedValues();
    StationaryObject update(splitOption);
    update.CopyObject(object);

    SelfUKF::updateResult result = MeasurementUpdate(update, error);
    if(result == RESULT_OUTLIER) setActive(false);
    else setActive(true);
    std::cout << summary(false) << std::endl;
    return;
}

void SelfUKF::InitialiseCachedValues()
{
    double alpha = 0.001;
    m_alpha_2 = alpha * alpha;
    m_beta = 2.0f;
    m_x = 0.0f;

    m_mean_weights = CalculateWeights(states_total, false);
    m_covariance_weights = CalculateWeights(states_total, false);

    Matrix sqrtOfProcessNoise = Matrix(3,3,true);
    sqrtOfProcessNoise[0][0] = 0.2; // Robot X coord.
    sqrtOfProcessNoise[1][1] = 0.2; // Robot Y coord.
    sqrtOfProcessNoise[2][2] = 0.005; // Robot Theta. 0.00001
    m_process_noise = sqrtOfProcessNoise * sqrtOfProcessNoise.transp();
    return;
}

Matrix SelfUKF::CalculateWeights(unsigned int num_states, bool covariance)
{
    const double alpha_2 = m_alpha_2;
    const double beta = m_beta;
    const double x = m_x;
    const unsigned int L = num_states;

    //const double lambda = alpha_2 * (L+x) - L;
    const double lambda = 1.0;

    unsigned int num_sigma_points = 2*L + 1;

    double weight_sum = 0.0;

    Matrix weights = Matrix(1,num_sigma_points, false);

    double meanWeight = lambda/(L+lambda);
    double outerWeight = 1.0/(2*(L+lambda));

    // First weight
    weights[0][0] = meanWeight;
    weight_sum += weights[0][0];
    if(covariance)
    {
         //weights[0][0] += (1 - alpha_2 + beta);
    }
    // The rest
    for(unsigned int i = 1; i < num_sigma_points; i++)
    {
        weights[0][i] = outerWeight;
        weight_sum += weights[0][i];
    }

    //assert(weight_sum == 1.0);
    return weights;
}

/*! @brief Time update
Performs a time update. The time update uses the oboemtry estimate and time information to
run the predictive stage of the kalman filter.

@param odometry The odmetry is represented by a three valued vector.
@param deltaTime The change in time elapsed since the previous time update.
*/
SelfUKF::updateResult SelfUKF::TimeUpdate(const std::vector<float>& odometry, OdometryMotionModel& motion_model, float deltaTime)
{
    float x = odometry[0];
    float y = odometry[1];
    float heading = odometry[2];

    // Step 1 : Calculate new sigma points based on previous covariance
    Matrix sigma_points = CalculateSigmaPoints(mean(), covariance());
    const unsigned int numSigmaPoints = sigma_points.getn();
    //-----------------------------------------------------------------------------------------------

    // Step 3: Update the state estimate - Pass all sigma points through motion model
    Pose2D oldPose, diffOdom;
    double *newPose;

    diffOdom.X = x;
    diffOdom.Y = y;
    diffOdom.Theta = heading;

    Matrix propogated_sigma_points(m_mean.getm(), numSigmaPoints, false);

    for (unsigned int i = 0 ; i < numSigmaPoints; i++)
    {
        oldPose.X = sigma_points[0][i];
        oldPose.Y = sigma_points[1][i];
        oldPose.Theta = sigma_points[2][i];

        newPose = motion_model.getNextSigma(diffOdom,oldPose);

        propogated_sigma_points[0][i] = *newPose;
        propogated_sigma_points[1][i] = *(newPose+1);
        propogated_sigma_points[2][i] = *(newPose+2);
    }

//    // Step 4: Calculate new state based on propagated sigma points and the weightings of the sigmaPoints
    Matrix newMean(m_mean.getm(),m_mean.getn(), false);

    for(unsigned int i = 0; i < numSigmaPoints; ++i)
    {
        newMean = newMean + m_mean_weights[0][i] * propogated_sigma_points.getCol(i) ;
    }

    Matrix covariance(states_total, states_total, false);
    Matrix diff;
    for(unsigned int i = 0; i < numSigmaPoints; ++i)
    {
        diff = propogated_sigma_points.getCol(i) - newMean;
        covariance = covariance + m_covariance_weights[0][i]*diff*diff.transp();
    }

    Matrix new_mean = newMean;
    Matrix new_covariance = covariance + m_process_noise;

    setMean(new_mean);
    setCovariance(new_covariance);

    return RESULT_OK;
}


/*! @brief  Multiple object update
Performs a simultaneous update for N landmarks.

@param locations The location of the landmarks seen. Nx2 Matrix.
@param measurements The measurements obtained to these landmarks. Nx2 Matrix.
@param R_Measurement The measurment noise of the updates. Nx2 Matrix.

@return The result of the update. RESULT_OK if update was successful, RESULT_OUTLIER if the update was ignored.
*/
SelfUKF::updateResult SelfUKF::MultipleObjectUpdate(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement)
{
    return RESULT_OUTLIER;
    /*
    unsigned int numObs = measurements.getm();
    const float c_threshold2 = 15.0f;
    Matrix R_obj_rel(R_Measurement);        // R = S^2
    Matrix S_obj_rel(cholesky(R_obj_rel));  // R = S^2

    // Unscented KF Stuff.
    Matrix yBar;                            //reset
    Matrix Py;
    Matrix Pxy=Matrix(3, 2*numObs, false);
    Matrix scriptX = CalculateSigmaPoints();
    const unsigned int numSigmaPoints = scriptX.getn();
          //----------------------------------------------------------------
    Matrix scriptY = Matrix(numObs, numSigmaPoints, false);
    Matrix temp = Matrix(numObs, 1, false);

    double dX,dY;

    for(unsigned int i = 0; i < numSigmaPoints; i++)
    {
        for(unsigned int j=0; j < numObs; j+=2)
        {
            dX = locations[j][0] - scriptX[0][i];
            dY = locations[j+1][0] - scriptX[1][i];
            temp[j][0] = sqrt(dX*dX + dY*dY);
            temp[j+1][0] = mathGeneral::normaliseAngle(atan2(dY,dX) - scriptX[2][i]);
        }
        scriptY.setCol(i, temp.getCol(0));
    }

    Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
    Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
    for(unsigned int i = 0; i < numSigmaPoints; i++)
    {
        Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
        My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
    }

    Matrix M1 = sqrtOfTestWeightings;
    yBar = My * M1.transp(); // Predicted Measurement.
    Py = (My - yBar * M1) * (My - yBar * M1).transp();
    Pxy = (Mx - m_mean * M1) * (My - yBar * M1).transp();

    Matrix invPyRObj = InverseMatrix(Py + R_obj_rel);

    Matrix K = Pxy * invPyRObj; // K = Kalman filter gain.

    Matrix y = Matrix(measurements); // Measurement. I terms of relative (x,y).

    //end of standard ukf stuff
    //RHM: 20/06/08 Outlier rejection.
    Matrix yDiffTemp = (yBar - y);

    for(unsigned int i = 0; i < (unsigned int)yDiffTemp.getm(); i+=2)
    {
        Matrix inv(2,2,false);
        inv[0][0] = invPyRObj[i][i];
        inv[0][1] = invPyRObj[i][i+1];
        inv[1][0] = invPyRObj[i+1][i];
        inv[1][1] = invPyRObj[i+1][i+1];

        Matrix diff(2,1,false);
        diff[0][0] = yDiffTemp[i][0];
        diff[1][0] = yDiffTemp[i+1][0];
        double innovation2 = convDble(diff.transp() * inv * diff);
        if(innovation2 > c_threshold2)
        {
            return RESULT_OUTLIER;
        }
    }

    // Update Alpha
    double innovation2measError = convDble((yBar - y).transp() * InverseMatrix(R_obj_rel) * (yBar - y));
    m_alpha *= 1 / (1 + innovation2measError);
    //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);

    m_covariance = HT( horzcat(Mx - m_mean*M1 - K*My + K*yBar*M1, K*S_obj_rel) );
    m_mean = m_mean - K*(yBar - y);

    return RESULT_OK;
    */
}

/*! @brief  Single object update
Performs an update for a single landmark.

@param object The landmark used for the update containing the relative measurement and location of the object.
@param error The measurment error.

@return The result of the update. RESULT_OK if update was successful, RESULT_OUTLIER if the update was ignored.
*/
//SelfUKF::updateResult SelfUKF::MeasurementUpdate(const StationaryObject& object, const MeasurementError& error)
//{
//    const float c_threshold2 = 15.0f;
//    // Calculate update uncertainties - S_obj_rel & R_obj_rel
//    Matrix S_obj_rel = Matrix(2,2,false);
//    S_obj_rel[0][0] = sqrt(error.distance());
//    S_obj_rel[1][1] = sqrt(error.heading());

//    Matrix R_obj_rel = S_obj_rel * S_obj_rel.transp(); // R = S^2

//    // Unscented KF Stuff.
//    Matrix yBar;                                  	//reset
//    Matrix Py;
//    Matrix Pxy = Matrix(, 2, false);                   //Pxy=[0;0;0];
//    Matrix scriptX = CalculateSigmaPoints();
//    const unsigned int numSigmaPoints = scriptX.getn();

//    Matrix scriptY = Matrix(2, numSigmaPoints, false);
//    Matrix temp = Matrix(2, 1, false);

//    for(unsigned int i = 0; i < numSigmaPoints; i++)
//    {
//        const double dX = object.X() - scriptX[0][i];
//        const double dY = object.Y() - scriptX[1][i];
//        temp[0][0] = sqrt(dX*dX + dY*dY);
//        temp[1][0] = mathGeneral::normaliseAngle(atan2(dY,dX) - scriptX[2][i]);
//        scriptY.setCol(i, temp.getCol(0));
//    }

//    Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
//    Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
//    for(unsigned int i = 0; i < numSigmaPoints; i++)
//    {
//        Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
//        My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
//    }

//    Matrix M1 = sqrtOfTestWeightings;
//    yBar = My * M1.transp(); // Predicted Measurement.
//    Py = (My - yBar * M1) * (My - yBar * M1).transp();
//    Pxy = (Mx - m_mean * M1) * (My - yBar * M1).transp();

//    Matrix K = Pxy * Invert22(Py + R_obj_rel); // K = Kalman filter gain.

//    Matrix y = Matrix(2,1,false); // Measurement. (Distance, heading).
//    y[0][0] = object.measuredDistance() * cos(object.measuredElevation());
//    y[1][0] = object.measuredBearing();

//    //end of standard ukf stuff
//    //RHM: 20/06/08 Outlier rejection.
//    double innovation2 = convDble((yBar - y).transp() * Invert22(Py + R_obj_rel) * (yBar - y));

//    // Update Alpha
//    double innovation2measError = convDble((yBar - y).transp() * Invert22(R_obj_rel) * (yBar - y));
//    m_alpha *= 1 / (1 + innovation2measError);
//    //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);

//    if (innovation2 > c_threshold2)
//    {
//        return RESULT_OUTLIER;
//    }

//    m_covariance = HT( horzcat(Mx - m_mean*M1 - K*My + K*yBar*M1, K*S_obj_rel) );
//    m_mean = m_mean - K*(yBar - y);
//    return RESULT_OK;
//}

SelfUKF::updateResult SelfUKF::MeasurementUpdate(const StationaryObject& object, const MeasurementError& error)
{
    const float c_threshold2 = 15.0f;
    Matrix y(2,1,false);
    y[0][0] = object.measuredDistance();
    y[1][0] = object.measuredBearing();

    Matrix measNoise = error.errorCovariance();

    Matrix current_mean = mean();

    Matrix stateEstimateSigmas = CalculateSigmaPoints(current_mean, covariance());

    const int numMeasurements = 2;
    const int numberOfSigmaPoints = stateEstimateSigmas.getn();

    Matrix Pyy(measNoise);
    Matrix Pxy(stateEstimateSigmas.getm(),numMeasurements,false);

    Matrix temp_pred(2,1,false);
    Matrix temp;

    Matrix yBar(2,1,false);
    // Calculate predicted measurement sigma points.
    Matrix projected_sigmas(yBar.getm(), stateEstimateSigmas.getn(), false);

    for(int i =0; i < numberOfSigmaPoints; i++)
    {
        const Matrix estimate = stateEstimateSigmas.getCol(i);
        const float dx = object.X() - estimate[states_x][0];
        const float dy = object.Y() - estimate[states_y][0];
        const float distance = sqrt(dx*dx + dy*dy);
        const float heading = mathGeneral::normaliseAngle(atan2(dy,dx) - estimate[states_heading][0]);
        temp_pred[0][0] = distance;
        temp_pred[1][0] = heading;
        projected_sigmas.setCol(i, temp_pred);

        yBar = yBar + m_mean_weights[0][i] * temp_pred;
    }
    for(int i =0; i < numberOfSigmaPoints; i++)
    {
        temp = projected_sigmas.getCol(i) - yBar;
        Pyy = Pyy + m_covariance_weights[0][i] * temp * temp.transp();
        Pxy = Pxy + m_covariance_weights[0][i] * (stateEstimateSigmas.getCol(i) - current_mean) * temp.transp();
    }
    Matrix temp_Pxy(m_mean.getm(), Pxy.getn(), false);
    for (unsigned int i = 0; i < temp_Pxy.getm(); ++i)
    {
        temp_Pxy.setRow(i,Pxy.getRow(i));
    }
    Pxy = temp_Pxy;

    Matrix K;
    if(numMeasurements == 2)
    {
        K = Pxy * Invert22(Pyy);
    }
    else
    {
        K = Pxy * InverseMatrix(Pyy);
    }

    //end of standard ukf stuff
    //RHM: 20/06/08 Outlier rejection.
    double innovation2 = convDble((yBar - y).transp() * Invert22(Pyy + measNoise) * (yBar - y));

    // Update Alpha
    double innovation2measError = convDble((yBar - y).transp() * Invert22(measNoise) * (yBar - y));

    m_alpha *= 1 / (1 + innovation2measError);

    if (innovation2 > c_threshold2)
    {
        return RESULT_OUTLIER;
    }

    Matrix innovation = (y - yBar);

    Matrix new_mean = mean() + K * innovation;
    Matrix new_covariance = covariance() - K*Pyy*K.transp();

    setMean(new_mean);
    setCovariance(new_covariance);
    return RESULT_OK;
}



/*!
    @brief  Perform update for Probabalistic Data Association Method
    The Probabalistic Data Association Method performs an single update performed by
    calculating a weighted sum of the correction, combining all possible options.

    @param object The ambiguous object containing the measruement to the object.
    @param possible_objects The possible stationary objects containing the potential
    positions of the observed object.

    @return Matrix containing the sigma points for the current model.
*/
SelfModel::updateResult SelfUKF::MeasurementUpdate(const AmbiguousObject& object, const std::vector<StationaryObject*>& possible_objects, const MeasurementError& error)
{
    /*
    const float c_threshold2 = 15.0f;
    Matrix y(2,1,false);
    y[0][0] = object.measuredDistance();
    y[1][0] = object.measuredBearing();

    Matrix measNoise = error.errorCovariance();

    Matrix current_mean = mean();

    Matrix stateEstimateSigmas = CalculateSigmaPoints(current_mean, covariance());

    const int numMeasurements = 2;
    const int numberOfSigmaPoints = stateEstimateSigmas.getn();

    Matrix Pyy(measNoise);
    Matrix Pxy(stateEstimateSigmas.getm(),numMeasurements,false);

    Matrix temp_pred(2,1,false);
    Matrix temp;

    Matrix yBar(2,1,false);
    // Calculate predicted measurement sigma points.
    Matrix projected_sigmas(yBar.getm(), stateEstimateSigmas.getn(), false);

    for(int i =0; i < numberOfSigmaPoints; i++)
    {
        const Matrix estimate = stateEstimateSigmas.getCol(i);
        const float dx = object.X() - estimate[states_x][0];
        const float dy = object.Y() - estimate[states_y][0];
        const float distance = sqrt(dx*dx + dy*dy);
        const float heading = mathGeneral::normaliseAngle(atan2(dy,dx) - estimate[states_heading][0]);
        temp_pred[0][0] = distance;
        temp_pred[1][0] = heading;
        projected_sigmas.setCol(i, temp_pred);

        yBar = yBar + m_mean_weights[0][i] * temp_pred;
    }
    for(int i =0; i < numberOfSigmaPoints; i++)
    {
        temp = projected_sigmas.getCol(i) - yBar;
        Pyy = Pyy + m_covariance_weights[0][i] * temp * temp.transp();
        Pxy = Pxy + m_covariance_weights[0][i] * (stateEstimateSigmas.getCol(i) - current_mean) * temp.transp();
    }
    Matrix temp_Pxy(m_mean.getm(), Pxy.getn(), false);
    for (unsigned int i = 0; i < temp_Pxy.getm(); ++i)
    {
        temp_Pxy.setRow(i,Pxy.getRow(i));
    }
    Pxy = temp_Pxy;

    Matrix K;
    if(numMeasurements == 2)
    {
        K = Pxy * Invert22(Pyy);
    }
    else
    {
        K = Pxy * InverseMatrix(Pyy);
    }

    //end of standard ukf stuff
    //RHM: 20/06/08 Outlier rejection.
    double innovation2 = convDble((yBar - y).transp() * Invert22(Pyy + measNoise) * (yBar - y));

    // Update Alpha
    double innovation2measError = convDble((yBar - y).transp() * Invert22(measNoise) * (yBar - y));

    m_alpha *= 1 / (1 + innovation2measError);

    if (innovation2 > c_threshold2)
    {
        return RESULT_OUTLIER;
    }

    Matrix innovation = (y - yBar);

    Matrix new_mean = mean() + K * innovation;
    Matrix new_covariance = covariance() - K*Pyy*K.transp();

    setMean(new_mean);
    setCovariance(new_covariance);
    return RESULT_OK;
    */
//    // Calculate update uncertainties - S_obj_rel & R_obj_rel
//    Matrix S_obj_rel = Matrix(2,2,false);
//    S_obj_rel[0][0] = sqrt(error.distance());
//    S_obj_rel[1][1] = sqrt(error.heading());

//    Matrix R_obj_rel = S_obj_rel * S_obj_rel.transp(); // R = S^2

//    // Unscented KF Stuff.
//    Matrix yBar;                                  	//reset
//    Matrix Py;
//    Matrix Pxy; //  = Matrix(7, 2, false);                   //Pxy=[0;0;0];
//    Matrix scriptX = CalculateSigmaPoints();
//    const unsigned int numSigmaPoints = scriptX.getn();

//    Matrix scriptY = Matrix(2, numSigmaPoints, false);
//    Matrix temp = Matrix(2, 1, false);

//    Matrix y = Matrix(2,1,false); // Measurement. (Distance, heading).
//    y[0][0] = object.measuredDistance() * cos(object.measuredElevation());
//    y[1][0] = object.measuredBearing();

//    Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
//    Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);

//    Matrix innovation;
//    std::vector<Matrix> innovations;
//    std::vector<Matrix> kf_gains;
//    std::vector<float> mvdists;
//    float mv_sum = 0.0f;

//    const float Pd = 0.95;  // Target detection probability
//    const float Pg = 0.95;  // Gate probability (the probability that the gate contains the true measurement if detected)

//    Matrix M1 = sqrtOfTestWeightings;

//    for (std::vector<StationaryObject*>::const_iterator obj_it = possible_objects.begin(); obj_it != possible_objects.end(); ++obj_it)
//    {
//        StationaryObject* object = (*obj_it);
//        // Calculate the expected measurement based on the current state.
//        for(unsigned int i = 0; i < numSigmaPoints; i++)
//        {
//            const double dX = object->X() - scriptX[0][i];
//            const double dY = object->Y() - scriptX[1][i];
//            temp[0][0] = sqrt(dX*dX + dY*dY);
//            temp[1][0] = mathGeneral::normaliseAngle(atan2(dY,dX) - scriptX[2][i]);
//            scriptY.setCol(i, temp.getCol(0));
//        }


//        for(unsigned int i = 0; i < numSigmaPoints; i++)
//        {
//            Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
//            My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
//        }

//        Matrix M1 = sqrtOfTestWeightings;
//        yBar = My * M1.transp(); // Predicted Measurement.
//        Py = (My - yBar * M1) * (My - yBar * M1).transp();
//        Pxy = (Mx - m_mean * M1) * (My - yBar * M1).transp();

//        Matrix Sk = Py + R_obj_rel;

//        // Calculate the Kalman filter gain.
//        Matrix K = Pxy * Invert22(Sk); // K = Kalman filter gain.
//        // Calculate the innovation.
//        innovation = yBar - y;

//        // Calculate Association Probability
//        float mv = MultiVariateNormalDistribution(Sk, y, yBar);
//        mv_sum += mv;
//        mvdists.push_back(mv);
//        innovations.push_back(innovation);
//        kf_gains.push_back(K);

////        //end of standard ukf stuff
////        //RHM: 20/06/08 Outlier rejection.
////        double innovation2 = convDble((yBar - y).transp() * Invert22(Py + R_obj_rel) * (yBar - y));

////        // Update Alpha
////        double innovation2measError = convDble((yBar - y).transp() * Invert22(R_obj_rel) * (yBar - y));
////        m_alpha *= 1 / (1 + innovation2measError);
////        //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);

////        if (innovation2 > c_threshold2)
////        {
////            return RESULT_OUTLIER;
////        }
//    }


//    // Combine the weighted innovations.
//    float weight_mult = 1 / (1-Pd*Pg + mv_sum);
//    float none_correct_weight = (1-Pd*Pg) * weight_mult;
//    Matrix combined_innovation(y.getm(), y.getn(), false);
//    Matrix kf_gain(kf_gains[0].getm(), kf_gains[0].getn(), false);

//    for(unsigned int index = 0; index < innovations.size(); ++index)
//    {
//        float weight = mvdists[index] * weight_mult;
//        combined_innovation = combined_innovation + weight * innovations[index];
//        kf_gain = kf_gain + weight*kf_gains[index];
//    }


//    // Update the model.
////    Matrix new_sqrtCovariance = HT( horzcat(Mx - m_mean*M1 - kf_gain*My + kf_gain*yBar*M1, kf_gain*S_obj_rel) );
////    setSqrtCovariance(new_sqrtCovariance);
////    Matrix new_mean = m_mean - kf_gain*combined_innovation;
////    setMean(new_mean);
//    return RESULT_OK;
}



/*! @brief  Calculation of sigma points
Calculates the sigma points for the current model state.
@return Matrix containing the sigma points for the current model.
*/
Matrix SelfUKF::CalculateSigmaPoints(Matrix mean, Matrix covariance) const
{
    const double alpha_2 = m_alpha_2;
    const double x = m_x;

    const unsigned int L = mean.getm();
    assert(L == covariance.getm());
    assert(L == covariance.getn());

    //const double lambda = alpha_2 * (L+x) - L;
    const double lambda = 1.0;

    int numberOfSigmaPoints = 2*L + 1;
    Matrix sigma_points(L, numberOfSigmaPoints, false);

    Matrix deviation;
    //Matrix param = m_numStates / (1-weights[0][0]) * m_covariance;
    Matrix param = (L + lambda) * covariance;
    Matrix sqtCovariance;

    sqtCovariance = cholesky(param);
    if(sqtCovariance.isValid() != true)
    {
        std::cout << "Bad matrix:" << std::endl;
        std::cout << param << std::endl;
        std::cout << "In model:" << std::endl;
        std::cout << summary(false) << std::endl;
        sqtCovariance = cholesky(param.transp());
        assert(sqtCovariance.isValid());
    }

    double sigmaAngleMax = 2.5;
    double min_angle = -sigmaAngleMax + mean[states_heading][0];
    double max_angle = sigmaAngleMax + mean[states_heading][0];

    unsigned int negIndex;
    sigma_points.setCol(0,mean); // First sigma point is the current mean with no deviation
    for(unsigned int i = 1; i < L+1; i++)
    {
        negIndex = i + L;
        deviation = sqtCovariance.getCol(i - 1);        // Get weighted deviation
        sigma_points.setCol(i, (mean + deviation));                // Add mean + deviation
        sigma_points.setCol(negIndex, (mean - deviation));  // Add mean - deviation
        // Crop heading
        sigma_points[states_heading][i] = mathGeneral::crop(sigma_points[states_heading][i], min_angle, max_angle);
        sigma_points[states_heading][negIndex] = mathGeneral::crop(sigma_points[states_heading][negIndex], min_angle, max_angle);
    }
    return sigma_points;
}


/*! @brief  Calculation of alpha weighting for the update
Calculates the alpha weighting adjustment factor depending on the current updates variation
from the current model.

@param innovation The innovation of the kalman filter update.
@param innovationVariance The variance for the innovation.
@param outlierLikelyhood The likelyhood of an outlier occuring.

@return The weighting factor ofthe current update.
*/
float SelfUKF::CalculateAlphaWeighting(const Matrix& innovation, const Matrix& innovationVariance, float outlierLikelyhood) const
{
    const int numMeas = 2;
    float notOutlierLikelyhood = 1.0 - outlierLikelyhood;
    float expRes = exp(-0.5*convDble(innovation.transp()*Invert22(innovationVariance)*innovation));
    float fracRes = 1.0 / ( sqrt( pow(2*mathGeneral::PI,numMeas)*determinant(innovationVariance) ) );
    return notOutlierLikelyhood * fracRes * expRes + outlierLikelyhood;
}

bool SelfUKF::clipState(int stateIndex, double minValue, double maxValue){
    bool clipped = false;
    if(m_mean[stateIndex][0] > maxValue){
        double mult, Pii;
        Matrix Si;
        Si = m_covariance.getRow(stateIndex);
        Pii = convDble(Si * Si.transp());
        mult = (m_mean[stateIndex][0] - maxValue) / Pii;
        m_mean = m_mean - mult * m_covariance * Si.transp();
        m_mean[stateIndex][0] = maxValue;
        clipped = true;
    }
    if(m_mean[stateIndex][0] < minValue){
        double mult, Pii;
        Matrix Si;
        Si = m_covariance.getRow(stateIndex);
        Pii = convDble(Si * Si.transp());
        mult = (m_mean[stateIndex][0] - minValue) / Pii;
        m_mean = m_mean - mult * m_covariance * Si.transp();
        m_mean[stateIndex][0] = minValue;
        clipped = true;
    }
    m_mean[states_heading][0] = mathGeneral::normaliseAngle(m_mean[states_heading][0]);
    if(clipped)
    {
        std::cout << "Cipping model: " << id() << std::endl;
    }
    return clipped;
}

std::ostream& operator<< (std::ostream& output, const SelfUKF& p_model)
{
    const SelfModel* model = static_cast<const SelfModel*>(&p_model);
    output << (*model);
    return output;
}

std::istream& operator>> (std::istream& input, SelfUKF& p_model)
{
    SelfModel* model = static_cast<SelfModel*>(&p_model);
    input >> (*model);
    return input;
}
