// File: CircleFitting.h

#ifndef _CircleFitting_h_DEFINED

#define _CircleFitting_h_DEFINED



#define MAX_ITERATIONS 5

#define MAX_PARLIMIT 208

#define FACTOR_UP 10

#define FACTOR_DOWN 0.04

#define EPSILON 0.5 

#define LMAEPSILON 0.00000005

#define MIN_POINT_DIST_FROM_EDGE 0

#define LAMBDA_MIN 0.0000000000000000000000000000000000000000000000000001

#define LAMBDA_MAX 1000.0


/*
struct Circle {

  double centreX, centreY;

  double radius, sd;

  bool isDefined;

};
*/
#include "Circle.h"


// Defines the stucture for a point

struct point {

  int x;

  int y;

};



enum {

  LEFTRIGHT = 0,

  RIGHTLEFT = 1,

  LEFTANDRIGHT = 2,

  TOPBOTTOM = 3,

  BOTTOMTOP = 4,

  TOPANDBOTTOM = 5

};

#include <vector>
#include "../Tools/Math/Vector2.h"
class CircleFitting {



  public:

    CircleFitting();

    ~CircleFitting();


    Circle FitCircleLMA(std::vector < Vector2<int> > points);

    //Circle FitCircleLMF(uint8* image, Blob* ballBlob, int direction);

    //Circle ThreePointFit(uint8*,point,point,point);

    int numFittedPoints; //  Stores the number of points found so far

    std::vector<point> fittedPoints; // Stores the points which have been found so far

    int meanX, meanY; 


  private:



    Circle AlgebraicCircleFit();

    Circle GeometricCircleFitLMA(Circle initialCircle);

    Circle GeometricCircleFitLMF(Circle initialCircle);

    Circle InvalidCircle();

    double Sigma (Circle circle);

    double Distance (Circle One, Circle Two);

    double GetDistance(double p1x, double p1y, double p2x, double p2y);

    double sumGSquared;

    double denominator;





    // Private data from Geometric Circle Fit

    double MaximumDeviation();

    bool Initalise(Circle);

    bool CholeskyDecomposition();

    bool ComputeMoments();

    bool UpdateCircleParameters();

    void GenerateCircle();

    inline bool CHECK_PIXEL(int pixel);





    double UpdateOldParameters();



    double lambda,oldA,oldF,oldT,newA,newF,newT,H,oldStandardDeviation,newStandardDeviation,centreSquared;

    double Xi,Yi,Zi,Ui,Vi,Gi,ADF,FACT,DGDAi,DGDFi,DGDTi;

    double H11,H12,H13,H22,H23,H33,F1,F2,F3,dA,dF,dT;

    double G11,G22,G33,G12,G13,G23,D1,D2,D3;

    double Xshift,Yshift,dX,dY,dMax;

    Circle finalCircle,oldCircle,newCircle;

    int colour1,colour2,colour3,colour4;





    // Three point fit 

    void Swap(point p1, point p2);

    bool GetCenter(point p1, point p2, point p3, point* center);

    double GetRadius(point p1, point p2, point p3);

};



#endif

