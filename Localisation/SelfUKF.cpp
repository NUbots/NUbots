#include "SelfUKF.h"
#include "Tools/Math/General.h"
#include <sstream>

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
SelfUKF::SelfUKF(const SelfUKF& source): SelfModel(source)
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
SelfUKF::SelfUKF(const SelfUKF& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time):
        SelfModel(parent, object, splitOption, time)
{
    InitialiseCachedValues();
    StationaryObject update(splitOption);
    update.CopyObject(object);

    SelfUKF::updateResult result = MeasurementUpdate(update, error);
    if(result == RESULT_OUTLIER) setActive(false);
    else setActive(true);
    return;
}

void SelfUKF::InitialiseCachedValues()
{
    const unsigned int numSigmaPoints = 2*states_total+1;

    // Create square root of W matrix
    sqrtOfTestWeightings = Matrix(1,numSigmaPoints,false);
    sqrtOfTestWeightings[0][0] = sqrt(c_Kappa/(states_total+c_Kappa));
    double outerWeighting = sqrt(1.0/(2*(states_total+c_Kappa)));
    for(int i=1; i <= 2*states_total; i++)
    {
      sqrtOfTestWeightings[0][i] = (outerWeighting);
    }

    CalculateSigmaWeights(c_Kappa);

    sqrtOfProcessNoise = Matrix(3,3,true);
    sqrtOfProcessNoise[0][0] = 0.2; // Robot X coord.
    sqrtOfProcessNoise[1][1] = 0.2; // Robot Y coord.
    sqrtOfProcessNoise[2][2] = 0.005; // Robot Theta. 0.00001

    return;
}

void SelfUKF::CalculateSigmaWeights(float kappa)
{
    unsigned int numPoints = 2*states_total + 1;
    m_sigmaWeights = Matrix(1,numPoints, false);
    m_sqrtSigmaWeights = Matrix(1,numPoints, false);

    double meanWeight = kappa/(states_total+kappa);
    double outerWeight = (1.0-meanWeight)/(2*states_total);

    // First weight
    m_sigmaWeights[0][0] = meanWeight;
    m_sqrtSigmaWeights[0][0] = sqrt(meanWeight);
    // The rest
    for(unsigned int i = 1; i < numPoints; i++)
    {
        m_sigmaWeights[0][i] = outerWeight;
        m_sqrtSigmaWeights[0][i] = sqrt(outerWeight);
    }
    return;
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
    Matrix sigmaPoints = CalculateSigmaPoints();
    const unsigned int numSigmaPoints = sigmaPoints.getn();
    //-----------------------------------------------------------------------------------------------

    // Step 3: Update the state estimate - Pass all sigma points through motion model
    Pose2D oldPose, diffOdom;
    double *newPose;

    diffOdom.X = x;
    diffOdom.Y = y;
    diffOdom.Theta = heading;

    cout << "initial sigma points" << std::endl << sigmaPoints << std::endl;

    for (unsigned int i = 0 ; i < numSigmaPoints; i++)
    {
        oldPose.X = sigmaPoints[0][i];
        oldPose.Y = sigmaPoints[1][i];
        oldPose.Theta = sigmaPoints[2][i];

        newPose = motion_model.getNextSigma(diffOdom,oldPose);

        sigmaPoints[0][i] = *newPose;
        sigmaPoints[1][i] = *(newPose+1);
        sigmaPoints[2][i] = *(newPose+2);
    }

    cout << "resulting sigma points" << std::endl << sigmaPoints << std::endl;

//    // Step 4: Calculate new state based on propagated sigma points and the weightings of the sigmaPoints
    Matrix newMean(m_mean.getm(),m_mean.getn(), false);

//    for(unsigned int i=0; i < numSigmaPoints; i++)
//    {
//        // Eqn 20
//        newMean = newMean + sqrtOfTestWeightings[0][i]*sqrtOfTestWeightings[0][i]*sigmaPoints.getCol(i);
//    }
//    //-----------------------------------------------------------------------------------------------

    newMean = sigmaPoints * m_sigmaWeights.transp();


//    // Step 5: Calculate measurement error and then find new srukfSx
//    Matrix newSrukfSx(m_covariance.getm(),m_covariance.getn(), false);
//    Matrix Mx(sigmaPoints.getm(),sigmaPoints.getn(), false);

//    for(unsigned int i=0; i < numSigmaPoints; i++)
//    {
//        // Eqn 21 part
//        Mx.setCol(i, sqrtOfTestWeightings[0][i] * (sigmaPoints.getCol(i) - newMean));      // Error matrix
//    }

    unsigned int numPoints = sigmaPoints.getn();
    Matrix covariance(states_total, states_total, false);
    Matrix diff;
    for(unsigned int i = 0; i < numPoints; ++i)
    {
        diff = sigmaPoints.getCol(i) - newMean;
        covariance = covariance + m_sigmaWeights[0][i]*diff*diff.transp();
    }

    cout << "timeupdate mean" << std::endl << m_mean << std::endl << "new mean" << std::endl << newMean << std::endl;
    cout << "timeupdate covariance" << std::endl << m_covariance << std::endl << "new covariance" << std::endl << HT(horzcat(covariance, sqrtOfProcessNoise*sqrtOfProcessNoise.transp())) << std::endl;

    m_covariance = HT(horzcat(covariance, sqrtOfProcessNoise*sqrtOfProcessNoise.transp()));
    m_mean = newMean;

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

    Matrix stateEstimateSigmas = CalculateSigmaPoints();

    const int numMeasurements = 2;
    const int numberOfSigmaPoints = stateEstimateSigmas.getn();

    Matrix measurementNoise = error.errorCovariance();
    Matrix Pyy(measurementNoise);
    Matrix Pxy(stateEstimateSigmas.getm(),numMeasurements,false);

    Matrix temp_pred(2,1,false);
    Matrix temp;

    Matrix y(2,1,false);
    y[0][0] = object.measuredDistance();
    y[1][0] = object.measuredBearing();

    Matrix yBar(2,1,false);
    const float dx = object.X() - m_mean[states_x][0];
    const float dy = object.Y() - m_mean[states_y][0];
    yBar[0][0] = sqrt(dx*dx + dy*dy);
    yBar[1][0] = mathGeneral::normaliseAngle(atan2(dy,dx) - m_mean[states_heading][0]);

    // Calculate predicted measurement sigma points.
    for(int i =0; i < numberOfSigmaPoints; i++)
    {
        const Matrix estimate = stateEstimateSigmas.getCol(i);
        const float dx = object.X() - estimate[states_x][0];
        const float dy = object.Y() - estimate[states_y][0];
        const float distance = sqrt(dx*dx + dy*dy);
        const float heading = mathGeneral::normaliseAngle(atan2(dy,dx) - estimate[states_heading][0]);
        temp_pred[0][0] = distance;
        temp_pred[1][0] = heading;
        temp = temp_pred - yBar;
        Pyy = Pyy + m_sigmaWeights[0][i] * temp * temp.transp();
        Pxy = Pxy + m_sigmaWeights[0][i] * (stateEstimateSigmas.getCol(i) - m_mean) * temp.transp();
    }

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
    double innovation2 = convDble((yBar - y).transp() * Invert22(Pyy + measurementNoise) * (yBar - y));

    // Update Alpha
    double innovation2measError = convDble((yBar - y).transp() * Invert22(measurementNoise) * (yBar - y));

    m_alpha *= 1 / (1 + innovation2measError);
    //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);

    if (innovation2 > c_threshold2)
    {
        return RESULT_OUTLIER;
    }

    m_mean = m_mean + K * (y - yBar);
    m_covariance = m_covariance - K*Pyy*K.transp();

    cout << "m_mean" << std::endl << m_mean << std::endl;

    cout << "covariance " << std::endl << m_covariance << std::endl;

    return RESULT_OK;
}

/*! @brief  Calculation of sigma points
Calculates the sigma points for the current model state.
@return Matrix containing the sigma points for the current model.
*/
Matrix SelfUKF::CalculateSigmaPoints() const
{
    int numberOfSigmaPoints = 2*states_total + 1;
    Matrix sigmaPoints(m_mean.getm(), numberOfSigmaPoints, false);

    const Matrix weights = m_sigmaWeights;

    sigmaPoints.setCol(0,m_mean); // First sigma point is the current mean with no deviation
    Matrix deviation;
    Matrix sqtCovariance = cholesky(m_numStates / (1-weights[0][0]) * m_covariance);

    double sigmaAngleMax = 2.5;
    double min_angle = -sigmaAngleMax + m_mean[states_heading][0];
    double max_angle = sigmaAngleMax + m_mean[states_heading][0];

    unsigned int negIndex;
    for(unsigned int i = 1; i < states_total+1; i++)
    {
        negIndex = i + states_total;
        deviation = sqtCovariance.getCol(i - 1);        // Get weighted deviation
        sigmaPoints.setCol(i, (m_mean + deviation));                // Add mean + deviation
        sigmaPoints.setCol(negIndex, (m_mean - deviation));  // Add mean - deviation
        // Crop heading
        sigmaPoints[states_heading][i] = mathGeneral::crop(sigmaPoints[states_heading][i], min_angle, max_angle);
        sigmaPoints[states_heading][negIndex] = mathGeneral::crop(sigmaPoints[states_heading][negIndex], min_angle, max_angle);
    }
    return sigmaPoints;

//    const unsigned int numSigmaPoints = 2 * states_total + 1;
//    Matrix sigmaPoints = Matrix(m_mean.getm(), numSigmaPoints, false);
//    sigmaPoints.setCol(0, m_mean);

//    //----------------Saturate ScriptX angle sigma points to not wrap
//    double sigmaAngleMax = 2.5;
//    unsigned int neg_index;
//    for(int i=1;i<states_total+1;i++)
//    {//hack to make test points distributed
//        neg_index = states_total + i;
//        // Addition Portion.
//        sigmaPoints.setCol(i, m_mean + sqrt((double)states_total + c_Kappa) * m_covariance.getCol(i - 1));
//        // Crop heading
//        sigmaPoints[states_heading][i] = mathGeneral::crop(sigmaPoints[states_heading][i], (-sigmaAngleMax + m_mean[states_heading][0]), (sigmaAngleMax + m_mean[states_heading][0]));
//        // Subtraction Portion.
//        sigmaPoints.setCol(neg_index, m_mean - sqrt((double)states_total + c_Kappa) * m_covariance.getCol(i - 1));
//        // Crop heading
//        sigmaPoints[states_heading][neg_index] = mathGeneral::crop(sigmaPoints[states_heading][neg_index], (-sigmaAngleMax + m_mean[states_heading][0]), (sigmaAngleMax + m_mean[states_heading][0]));
//    }
//    return sigmaPoints;
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

std::string SelfUKF::summary(bool brief) const
{
    std::stringstream buffer;
    buffer << "Model Id: " << id() << " Active: " << active() << " Alpha: " << alpha() << " Position: (" << mean(states_x) << ",";
    buffer << mean(states_y) << "," << mean(states_heading) << ")" << std::endl;
    if(!brief)
    {
            buffer << "Variance - Position: (" << sd(states_x)*sd(states_x) << ",";
            buffer << sd(states_y)*sd(states_y) << "," << sd(states_heading)*sd(states_heading) << endl;
    }
    return buffer.str();
}
