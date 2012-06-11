/*! @file CircleFitting.cpp

    @brief Circle Fitting for ball and other circular objects.

    @author Chris S and Craig M (2005), Las Modified: Aaron Wong (2010)
 
    @ref "Extentions to Vision in Four legged league"
 Copyright (c) 2010 Aaron Wong
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */
// File: CircleFitting.cpp
// Date: 30/03/03
//

#include <math.h> 
#include <float.h>
#include "CircleFitting.h"
#include "../Tools/Math/General.h"
#include <stdlib.h>

#if TARGET_OS_IS_WINDOWS
    #include <QDebug>
#endif

using namespace mathGeneral;

const double EXTRA_DISTANCE_FRACTION = 0;//1.0/10.0;

const int MAX_POINTS = 40;



// Class constructor

CircleFitting::CircleFitting() 
{ 
    fittedPoints.reserve(20); //RESERVE 20 SPOTs of MEMORY for ballPoints.
}


CircleFitting::~CircleFitting() 
{

}

inline bool CircleFitting::CHECK_PIXEL(int pixel)
{

  return ((pixel==colour1)||(pixel==colour2)||(pixel==colour3)||(pixel==colour4));
}


Circle CircleFitting::FitCircleLMA(std::vector < Vector2<int> > points)
{
    numFittedPoints = points.size();
    fittedPoints.empty();
    for(int i = 0; i < numFittedPoints; i++)
    {
         point tempPoint;
         tempPoint.x = points[i].x;
         tempPoint.y = points[i].y;
         fittedPoints.push_back(tempPoint);
    }

    if (numFittedPoints > 5) 
    {
        Circle algebraicCircle = AlgebraicCircleFit(); //! Generates an approximate centre using an algebraic approximation to the centre of the circle
        if (algebraicCircle.isDefined) 
        {
            #if TARGET_OS_IS_WINDOWS
                qDebug() << "Algebraic Fit Result: "<< "Circle found " << algebraicCircle.isDefined<<": (" << algebraicCircle.centreX << "," << algebraicCircle.centreY << ") Radius: "<< algebraicCircle.radius << " Fitting: " << algebraicCircle.sd<< endl;;
            #endif
            Circle geometricCircle = GeometricCircleFitLMA(algebraicCircle); //! Uses the algebraic approximation to generate a more accurate centre and radius using geometric fitting
            #if TARGET_OS_IS_WINDOWS
                qDebug() << "Tried Geometric Fit Result: "<< "Circle found " << geometricCircle.isDefined<<": (" << geometricCircle.centreX << "," << geometricCircle.centreY << ") Radius: "<< geometricCircle.radius << " Fitting: " << geometricCircle.sd<< endl;;
            #endif
            if(geometricCircle.isDefined == true)
            {
                return geometricCircle;
            }
        }
        return algebraicCircle;
    }
    // If we make it to this point we cannot return a circle
    return InvalidCircle();  
}


Circle CircleFitting::FitCircleLMF(std::vector < Vector2<int> > points)
{
    numFittedPoints = points.size();
    fittedPoints.clear();
    for(int i = 0; i < numFittedPoints; i++)
    {
         point tempPoint;
         tempPoint.x = points[i].x;
         tempPoint.y = points[i].y;
         fittedPoints.push_back(tempPoint);
    }

    if (numFittedPoints > 5)
    {
        Circle algebraicCircle = AlgebraicCircleFit(); // Generates an approximate centre using an algebraic approximation to the centre of the circle

        if (algebraicCircle.isDefined)
        {
            #if TARGET_OS_IS_WINDOWS
                qDebug() << "Algebraic Fit Result: "<< "Circle found " << algebraicCircle.isDefined<<": (" << algebraicCircle.centreX << "," << algebraicCircle.centreY << ") Radius: "<< algebraicCircle.radius << " Fitting: " << algebraicCircle.sd<< endl;;
            #endif

            Circle geometricCircle = GeometricCircleFitLMF(algebraicCircle); // Uses the algebraic approximation to generate a more accurate centre and radius using geometric fitting
            #if TARGET_OS_IS_WINDOWS
                qDebug() << "Tried Geometric Fit Result: "<< "Circle found " << geometricCircle.isDefined<<": (" << geometricCircle.centreX << "," << geometricCircle.centreY << ") Radius: "<< geometricCircle.radius << " Fitting: " << geometricCircle.sd<< endl;;
            #endif

            return geometricCircle;
        }
    }

    // If we make it to this point we cannot return a circle
    return InvalidCircle();

}



// Returns an invalid circle which can be returned in cases where an error condition occurs

Circle CircleFitting::InvalidCircle() {

  Circle invalidCircle;

  invalidCircle.isDefined = false;

  return invalidCircle;  

}





// Performs an algebraic fit to the points already stored in the fitted points array. 
// This gives us a rough approximation for the centre which we later use in the geometric Circle fit
Circle CircleFitting::AlgebraicCircleFit()
{
    double sumXY,sumXX,sumYY,sumXZ,sumYZ;
    double meanXY,meanXX,meanYY,meanXZ,meanYZ;
    double B,C,G11,G12,G22,D1,D2;

    // Routine that shifts the data to the center of mass (i.e. 0.0)
    meanX = meanY = 0; // The mean values represent the offset from the original postion in the image
    //std::vector<point> centredPoints;
    //centredPoints.reserve(fittedPoints.size());

    // Sum all the elements in the array
    for (int i=0; i < numFittedPoints; i++)
    {
        meanX += fittedPoints[i].x;
        meanY += fittedPoints[i].y;
    }

    // Calculate the average
    meanX = (int) meanX/numFittedPoints;
    meanY = (int) meanY/numFittedPoints;

    // Shift the data to the mean
    for (int j=0; j < numFittedPoints; j++)
    {
        fittedPoints[j].x -= meanX;
        fittedPoints[j].y -= meanY;
    }

    // Now that all data has been shifted toward the centre we can continue
    // computing moments (note: all moments are normed, i.e. divided by N)
    double Xi,Yi,Zi;
    sumXY=sumXX=sumYY=sumXZ=sumYZ=0;

    for (int k=0; k < numFittedPoints; k++)
    {
        Xi = fittedPoints[k].x;
        Yi = fittedPoints[k].y;
        Zi = Xi*Xi + Yi*Yi;

        sumXY = sumXY + Xi*Yi;
        sumXX = sumXX + Xi*Xi;
        sumYY = sumYY + Yi*Yi;
        sumXZ = sumXZ + Xi*Zi;
        sumYZ = sumYZ + Yi*Zi;
    }

    meanXX = sumXX/numFittedPoints;
    meanYY = sumYY/numFittedPoints;
    meanXY = sumXY/numFittedPoints;
    meanXZ = sumXZ/numFittedPoints;
    meanYZ = sumYZ/numFittedPoints;

    // computing the circle parameters
    G11 = sqrt(meanXX);

    // Check that we have not generated an infinity error
    if ((G11 < DBL_MIN) || (G11 > DBL_MAX))
        return InvalidCircle();
  
    G12 = meanXY/G11;
    // Check that the value to be squared rooted is not negative
    if ((meanYY - G12*G12) < 0)
        return InvalidCircle();

    G22 = sqrt(meanYY - G12*G12);

    // Checking that we have not generated an infinity error
    if ((G22 < DBL_MIN) || (G22 > DBL_MAX))
        return InvalidCircle();

    D1 = meanXZ/G11;
    D2 = (meanYZ - D1*G12)/G22;
    C = D2/G22;
    B = (D1 - G12*C)/G11;

    Circle algebraicCircle;             // Defines the circle which is returned as part the result from the method
    algebraicCircle.centreX = B/2;      // Sets the x coordinate of the centre of the circle
    algebraicCircle.centreY = C/2;      // Sets the y coordinate of the centre of the circle
    algebraicCircle.radius = sqrt(pow(algebraicCircle.centreX,2) + pow(algebraicCircle.centreY,2) + meanXX+meanYY); // Determine the radius of the centre of the circle
    algebraicCircle.sd = Sigma(algebraicCircle);
    algebraicCircle.isDefined = true;

    return algebraicCircle;
}



// LMABEST - Note computationally expensive
Circle CircleFitting::GeometricCircleFitLMA(Circle initialCircle)
{
    int iteration;
    Xshift=0.0;
    Yshift=0.0;
    dX=0.0;
    dY=0.0;
    dMax=0.0;

    // Calculate the maximum standard deviation
    dMax = MaximumDeviation();
 
    if (!Initalise(initialCircle))
    {
        #ifdef TARGET_OS_IS_WINDOWS
            qDebug() << "GemonetricCircle: Failed to Initialise";
        #endif
        return InvalidCircle();
    }

    // initializing lambda and iteration
    lambda = 0.01; // Note 0.01 is arbitararily small number from which to start from
    iteration = 0;

    while(true)
    {
        #ifdef TARGET_OS_IS_WINDOWS
        qDebug() << "Geometric Itteration: " << iteration << "\t Defined: " << newCircle.isDefined << "\t(" << newCircle.centreX << "," << newCircle.centreY <<")"
                 << "\t Radius:" << newCircle.radius << "\tRMSE: " << newCircle.sd;
        #endif

        //Assigning Previous interration as old:
        oldA = newA;
        oldF = newF;
        oldT = newT;
        oldStandardDeviation = newStandardDeviation;
        oldCircle = newCircle;

        shiftXY:
            iteration++;
            if(iteration > MAX_ITERATIONS)
                break;

            ComputeMoments(); // computing matrices

        try_again:
            CholeskyDecomposition();

        if ((1.0+4.0*newA*newF < LMAEPSILON) && (lambda > 1.0))
        {
            double bTemp = UpdateOldParameters();
            if (bTemp > 0)
            {
                oldT = 2.0*PI - oldT;
            }
            goto shiftXY;
        }

        if (1.0+4.0*newA*newF < LMAEPSILON)
        {
            lambda *= FACTOR_UP;
            if ((iteration > MAX_ITERATIONS) || (lambda > LAMBDA_MAX))
                break;
            else
                goto try_again;
        }
        UpdateCircleParameters();

        // checking if improvement is gained

        if (newStandardDeviation <= oldStandardDeviation)
        {    //   yes, improvement
            if (Distance(newCircle,oldCircle) < LMAEPSILON)
            {
                oldA = newA;
                oldF = newF;
                oldT = newT;

                while (oldT > 2.0*PI)
                {
                    oldT-=2.0*PI;
                }

                while (oldT < 0.0)
                {
                    oldT+=2.*PI;
                }

                oldStandardDeviation = newStandardDeviation;

                break;
            }

            lambda *= FACTOR_DOWN;

            if (lambda < LAMBDA_MIN)
            {
                lambda=LAMBDA_MIN;
            }

            continue;

        }
        else
        { //   no improvement

            lambda *= FACTOR_UP;

            if ((iteration > MAX_ITERATIONS) ||(lambda > LAMBDA_MAX))
            {
                break;
            }
            else
            {
                goto try_again;
            }

        }

    }

    GenerateCircle();

    return finalCircle;

}



///////////////////////////////////////////////////////////////////////////////////////



// Calculates the maximum standard deviation for the set of points

double CircleFitting::MaximumDeviation() {

  int imax,jmax;

  double Xi,Yi,dMax=0.0;

  for (int i=1; i<numFittedPoints; i++) {

    for (int j=0; j<i; j++) {

      Xi = fittedPoints[i].x - fittedPoints[j].x;

      Yi = fittedPoints[i].y - fittedPoints[j].y;

      if (dMax < (Xi*Xi+Yi*Yi)) {

        dMax=Xi*Xi+Yi*Yi;

        imax = i;

        jmax = j;

      }

    }

  }

  dMax = sqrt(dMax);

  return dMax;

}



// Need to determine reasonable values for the radius and the centre at various distances and cap the inputs to these values
// As the algorithm runs generally for all values it must be a bad prediction from the algebraic approximation that is causing the issues.
bool CircleFitting::Initalise(Circle initialCircle) {

  // Starting with the initial guess provided by the algebraic technique
  if ((initialCircle.radius + initialCircle.radius) <= 0) 
    return false;
  
  newA = 1.0/(initialCircle.radius + initialCircle.radius);
  centreSquared = initialCircle.centreX*initialCircle.centreX + initialCircle.centreY*initialCircle.centreY;

  newF = (centreSquared - initialCircle.radius*initialCircle.radius)*newA;

  // Check that the value that we're using is valid
  //double temp = -initialCircle.centreX/sqrt(centreSquared);
  /*
  qDebug() << "Setting temp: " << temp << DBL_MIN << DBL_MAX;
  if ((temp < DBL_MIN) || (temp > DBL_MAX))

    return false;
    */


  // Check that we are not generating a divide by zero error
  //qDebug() << "Setting centreSquared: " << centreSquared;
  if (sqrt(centreSquared) <= 0) 
    return false;

  newT = acos(-initialCircle.centreX/sqrt(centreSquared));
  if (initialCircle.centreY > 0) newT = 2.0*PI - newT;

  newStandardDeviation = initialCircle.sd;
  newCircle = initialCircle;

  for (int k=0; k<1000; k++)
  { // Perhaps this 1000 could be changed to numFittedPoints i.e. 1000 was used because there was a 1000 samples in the original experiment
    if ((1.0+4.0*newA*newF) >= LMAEPSILON)
    {
        dX=(rand()-0.5)*dMax;
        dY=(rand()-0.5)*dMax;
        Xshift += dX;
        Yshift += dY;
        newCircle.centreX += dX;
        newCircle.centreY += dY;

        centreSquared = newCircle.centreX*newCircle.centreX + newCircle.centreY*newCircle.centreY;
        newF = (centreSquared - newCircle.radius*newCircle.radius)*newA;

        // Check that we are not generating a divide by zero error
        if (sqrt(centreSquared) <= 0)
            return false;

        newT = acos(((double)-newCircle.centreX)/sqrt(centreSquared));

        if (newCircle.centreY > 0)
            newT = 2.*PI - newT;
    }
    else
      break;
  }
  return true; // True indicates that the initalisation was successful
}



bool CircleFitting::ComputeMoments() {

  // computing moments



  H11=H12=H13=H22=H23=H33=F1=F2=F3=0.0;

 

  for (int l=0; l<numFittedPoints; l++) {

    Xi = fittedPoints[l].x + Xshift;

    Yi = fittedPoints[l].y + Yshift;

    Zi = Xi*Xi + Yi*Yi;

    Ui = Xi*cos(oldT) + Yi*sin(oldT);

    Vi =-Xi*sin(oldT) + Yi*cos(oldT);

     

    // Check that we are not generating a negative square root error - note the equal to check is for the divide which is down further 

    if ((1.0 + 4.0*oldA*oldF) <= 0) 

      return false; 



    H = sqrt(1.0 + 4.0*oldA*oldF);

    ADF = oldA*Zi + H*Ui + oldF;



    // Check that we are not generating a negative square root error - note the equal to check is for the divide which is down further 

    if ((1.0 + 4.0*oldA*ADF) <= 0) 

      return false; 

    

    Gi = 2.0*ADF/(sqrt(1.0 + 4.0*oldA*ADF) + 1.0);

    FACT = 2.0/(sqrt(1.0 + 4.0*oldA*ADF) + 1.0)*(1.0 - oldA*Gi/sqrt(1.0 + 4.0*oldA*ADF));

    DGDAi = FACT*(Zi + 2.0*oldF*Ui/H) - Gi*Gi/sqrt(1.0 + 4.0*oldA*ADF);

    DGDFi = FACT*(2.0*oldA*Ui/H + 1.0);

    DGDTi = FACT*H*Vi; 

     

    H11 += DGDAi*DGDAi;

    H12 += DGDAi*DGDFi;

    H13 += DGDAi*DGDTi;

    H22 += DGDFi*DGDFi;

    H23 += DGDFi*DGDTi;

    H33 += DGDTi*DGDTi;

     

    F1 += Gi*DGDAi;

    F2 += Gi*DGDFi;

    F3 += Gi*DGDTi;

  }

  return true;

}



bool CircleFitting::CholeskyDecomposition() {

    // Cholesky decomposition



    // Check that we are not generating a negative square root error - note the equal to check is for the divide which is down further 

    if ((H11 + lambda) <= 0) 

      return false; 



    G11 = sqrt(H11 + lambda);

    G12 = H12/G11;

    G13 = H13/G11;



    // Check that we are not generating a negative square root error - note the equal to check is for the divide which is down further 

    if ((H22 + lambda - G12*G12) <= 0) 

      return false; 



    G22 = sqrt(H22 + lambda - G12*G12);

    G23 = (H23 - G12*G13)/G22;



    // Check that we are not generating a negative square root error - note the equal to check is for the divide which is down further 

    if ((H33 + lambda - G13*G13 - G23*G23) <= 0) 

      return false; 



    G33 = sqrt(H33 + lambda - G13*G13 - G23*G23);



    D1 = F1/G11;

    D2 = (F2 - G12*D1)/G22;

    D3 = (F3 - G13*D1 - G23*D2)/G33;

    

    // Calculates the change in the three parameters

    dT = D3/G33;

    dF = (D2 - G23*dT)/G22;

    dA = (D1 - G12*dF - G13*dT)/G11;

      

    // Updating the parameters by subtracting the difference from the old parameter

    newA = oldA - dA;

    newF = oldF - dF;

    newT = oldT - dT;

    return true;

}





bool CircleFitting::UpdateCircleParameters() {
  sumGSquared = 0.0;
  for (int i=0; i < numFittedPoints; i++)
  {
      Xi = fittedPoints[i].x + Xshift;
      Yi = fittedPoints[i].y + Yshift;
      Zi = Xi*Xi + Yi*Yi;
      Ui = Xi*cos(newT) + Yi*sin(newT);

      // Check that we are not generating a negative square root error - note the equal to check is for the divide which is down further
      if ((1.0 + 4.0*newA*newF) <= 0) 
        return false; 



      ADF = newA*Zi + (sqrt(1.0 + 4.0*newA*newF))*Ui + newF;
      denominator = sqrt(4.0*newA*ADF + 1.0) + 1.0;
      
      if (denominator <= 0)
        return false;

      Gi = 2.0*ADF/(denominator);
      sumGSquared = sumGSquared+Gi*Gi;
  }

  newStandardDeviation = sqrt(sumGSquared/numFittedPoints);
  H = sqrt(1.0+4.0*newA*newF);

  double denominator = newA + newA;
  newCircle.centreX = -H*cos(newT)/(denominator) - Xshift;
  newCircle.centreY = -H*sin(newT)/(denominator) - Yshift;
  newCircle.radius = 1.0/fabs(denominator);
  newCircle.sd = newStandardDeviation;

  return true;
}



void CircleFitting::GenerateCircle() {

  H = sqrt(1.0 + 4.0*oldA*oldF);
  finalCircle.centreX = -H*cos(oldT)/(oldA+oldA) - Xshift;
  finalCircle.centreY = -H*sin(oldT)/(oldA+oldA) - Yshift;
  finalCircle.radius = 1.0/fabs(oldA+oldA);
  finalCircle.sd = oldStandardDeviation;
  return;

}



double CircleFitting::UpdateOldParameters() {
      dX=(rand()-0.5)*dMax;
      dY=(rand()-0.5)*dMax;

      Xshift += dX;
      Yshift += dY;

      H = sqrt(1.+4.*oldA*oldF);
      double aTemp = -H*cos(oldT)/(oldA+oldA) + dX;
      double bTemp = -H*sin(oldT)/(oldA+oldA) + dY;
      double rTemp = 1.0/fabs(oldA+oldA);

      oldA = 1.0/(rTemp + rTemp);
      centreSquared = aTemp*aTemp + bTemp*bTemp;
      oldF = (centreSquared - rTemp*rTemp)*oldA;
      oldT = acos(-aTemp/sqrt(centreSquared));

      return bTemp;
}



 

// The following method performs a geometric circle fit using the Levenberg-Marquard (LMF) algorithm and the points
// provided in the fitted points array
Circle CircleFitting::GeometricCircleFitLMF(Circle initialCircle) {
  double dx,dy,radius,u,v;
  double UUl,VVl,Nl,F1,F2,F3,dX,dY,dR;
  double G11,G22,G33,G12,G13,G23,D1,D2,D3;
  double meanU, meanV, meanUSquared, meanVSquared, meanUV, meanR;
  double sumU, sumV, sumUSquared, sumVSquared,sumUV, sumR;
  Circle oldCircle,newCircle;

  // starting with the given initial guess as determined by the algebraic approximation
  newCircle = initialCircle;

  // initializing the lambda and iteration counters
  double lambda = 1.0;
  int iteration = 0;

  while(true) {
    oldCircle = newCircle; // Updates the pointer to point to the newest circle which has been determined
    if (iteration > MAX_ITERATIONS)
    {
      oldCircle.centreX = oldCircle.centreX + meanX;
      oldCircle.centreY = oldCircle.centreY + meanY;
      oldCircle.isDefined = true;
      //qDebug() << "GeometricFit: MaxIterations";
      return oldCircle;
    }
    // computing moments
    sumU=sumV=sumUSquared=sumVSquared=sumUV=sumR=0;

    // Iterate over the array, summing the difference between the original points and the lastest approximation
    for (int i=0; i < numFittedPoints; i++)
    {
      dx = fittedPoints[i].x - oldCircle.centreX;
      dy = fittedPoints[i].y - oldCircle.centreY;
      radius = sqrt(dx*dx + dy*dy);

      // Check that the radius is defined
      if (radius <= 0)
      {
        //qDebug() << "GeometricFit: Invalid Circle; Radius less then 0";
        return InvalidCircle();
      }
      u = dx/radius;
      v = dy/radius;
      sumU = sumU + u;
      sumV = sumV + v;
      sumUSquared = sumUSquared + u*u;
      sumVSquared = sumVSquared + v*v;
      sumUV = sumUV + u*v;
      sumR = sumR + radius;
    }
    meanU = sumU / numFittedPoints;
    meanV = sumV / numFittedPoints;
    meanUSquared = sumUSquared / numFittedPoints;
    meanVSquared = sumVSquared  / numFittedPoints;
    meanUV =  sumUV / numFittedPoints;
    meanR = sumR  / numFittedPoints;

    // computing matrices
    F1 = oldCircle.centreX + oldCircle.radius*meanU;
    F2 = oldCircle.centreY + oldCircle.radius*meanV;
    F3 = oldCircle.radius - meanR;

    while (true)
    {
      UUl = meanUSquared + lambda;
      VVl = meanVSquared + lambda;
      Nl = 1.0 + lambda;

      //	Cholesly decomposition
      G11 = sqrt(UUl);

      // Check that G11 is defined
      if (G11 <= 0)
      {
            //qDebug() << "GeometricFit: G11 less then 0";
            return InvalidCircle();
      }
      G12 = meanUV/G11;
      G13 = meanU/G11;

      if ((VVl - G12*G12) < 0)
      {
            //qDebug() << "GeometricFit: VVl - G12*G12 < 0";
            return InvalidCircle();
      }
      G22 = sqrt(VVl - G12*G12);
      G23 = (meanV - G12*G13)/G22;

      if ((Nl - G13*G13 - G23*G23) < 0)
      {
        //qDebug() << "GeometricFit: Nl - G13*G13 - G23*G23 < 0";
        return InvalidCircle();
      }

      G33 = sqrt(Nl - G13*G13 - G23*G23);

      D1 = F1/G11;
      D2 = (F2 - G12*D1)/G22;
      D3 = (F3 - G13*D1 - G23*D2)/G33;

      dR = D3/G33;
      dY = (D2 - G23*dR)/G22;
      dX = (D1 - G12*dY - G13*dR)/G11;

      // updating the parameters
      newCircle.centreX = oldCircle.centreX - dX;
      newCircle.centreY = oldCircle.centreY - dY;
      newCircle.radius = oldCircle.radius - dR;
      newCircle.sd = Sigma(newCircle);

      if (fabs(newCircle.centreX) > MAX_PARLIMIT || fabs(newCircle.centreY) > MAX_PARLIMIT)
      {
        oldCircle.centreX = oldCircle.centreX + meanX;
        oldCircle.centreY = oldCircle.centreY + meanY;
        oldCircle.isDefined = true;
        //qDebug() << "GeometricFit: MAX PARLIMIT" << MAX_PARLIMIT << fabs(newCircle.centreX) << fabs(newCircle.centreY) ;
        return oldCircle;
      }

      iteration++;

      // Increment the iteration counter and check if the maximum number of iterations has been reached
      // If so return the last circle found
      if (newCircle.sd <= oldCircle.sd)
      {   //   yes, improvement
        if (Distance(newCircle,oldCircle) < EPSILON) {
          //qDebug() << "GeometricFit: EPSLION" << EPSILON << Distance(newCircle,oldCircle);
          oldCircle.centreX = oldCircle.centreX + meanX;
          oldCircle.centreY = oldCircle.centreY + meanY;
          oldCircle.isDefined = true;

          return oldCircle;
        }
        lambda *= FACTOR_DOWN;
        break;
      }	else {                      //   no improvement
        if (iteration > MAX_ITERATIONS) {
          oldCircle.centreX = oldCircle.centreX + meanX;
          oldCircle.centreY = oldCircle.centreY + meanY;
          oldCircle.isDefined = true;
          //qDebug() << "GeometricFit: ITERATION" << MAX_ITERATIONS<< iteration;
          return oldCircle;
        }
        lambda *= FACTOR_UP;
        continue;
      }
    }
  }
}

double CircleFitting::Distance (Circle One, Circle Two) {
	double dist = (fabs(One.centreX-Two.centreX) + fabs(One.centreY-Two.centreY) + fabs(One.radius-Two.radius))/(One.radius + Two.radius);
	return dist;
}



double CircleFitting::Sigma (Circle circle)
{
    double sum = 0.,dx,dy,di;
    for (int i=0; i<numFittedPoints; i++)
    {
        dx = fittedPoints[i].x - circle.centreX;
        dy = fittedPoints[i].y - circle.centreY;
        di = sqrt(dx*dx+dy*dy) - circle.radius;
        sum += di*di;
    }
    //qDebug() << "SUM ERROR (circ): " << sum << " Points: "  << numFittedPoints  << "\tMSE:" << sqrt(sum/numFittedPoints) ;
    return sqrt(sum/numFittedPoints);
}



// The following method attempts to calculate the radius and the centre using the three points provided 
// and by looking at the interestion of the bisectors
/*
Circle CircleFitting::ThreePointFit(uint8* image, point point1, point point2, point point3) {

  cImage = image;
  // point centre;
  // GetCenter(point1,point2,point3,&centre) // Determines the centre and checks whether we can generate a circle
  // resultCircle.centreX = centre.x;
  // resultCircle.centreY = centre.y;
  Circle resultCircle;
  resultCircle.centreX = -1;
  resultCircle.centreY = -1;  
  resultCircle.radius = GetRadius(point1,point2,point3);
  resultCircle.sd = -1;
  resultCircle.isDefined = true;
  return resultCircle;
}
*/

bool CircleFitting::GetCenter(point p1, point p2, point p3, point* center) { 
  double x=0,y=0;
  double ma=0,mb=0;
  bool result = true;

  // we don't want infinite slopes or 0 slope for line 1, since we'll divide by "ma" below 
  if ((p1.x == p2.x) || (p1.y == p2.y)) Swap(p2,p3);
  if (p2.x == p3.x) Swap(p1,p2);

  if (p1.x != p2.x) ma = ((p2.y-p1.y)/(p2.x-p1.x));
  else result = false;

  if (p2.x != p3.x) mb = ((p3.y-p2.y)/(p3.x-p2.x));
  else result = false;

  if ((ma == 0) && (mb == 0)) result = false;

  if (result) {
    x =(ma*mb*(p1.y-p3.y)+mb*(p1.x+p2.x)-ma*(p2.x+p3.x))/(2*(mb-ma));
    y =-(x-(p1.x+p2.x)/2)/ma + (p1.y+p2.y)/2;
    center->x=(int) x;
    center->y=(int) y;
  } 
  return result;
}



// Uses the distance R= (abc/4K) where a,b,c are the lengths of the three sides and K is the area
double CircleFitting::GetRadius(point p1, point p2, point p3) {
  // Return the radius of a circle defined by 3 points on it's circumference
  double s;
  double a,b,c;
  a = GetDistance(p1.x,p1.y,p2.x,p2.y); // Note this is just the distance formula
  b = GetDistance(p2.x,p2.y,p3.x,p3.y); 
  c = GetDistance(p3.x,p3.y,p1.x,p1.y); 
  s=(a+b+c)/2;
  return ((a*b*c)/(4*sqrt(s*(s-a)*(s-b)*(s-c))));
}

void CircleFitting::Swap(point p1, point p2) { 
  // Note the points should be changed to use pointers
  point temp;
  temp=p1; 
  p1=p2; 
  p2=temp;
}



double CircleFitting::GetDistance(double p1x, double p1y, double p2x, double p2y)
{
  double distX = p1x - p2x;
  double distY = p1y - p2y;
  return sqrt(distX*distX+distY*distY);
} 

