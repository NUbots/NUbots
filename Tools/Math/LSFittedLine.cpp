#include "LSFittedLine.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <limits>
using namespace std;

LSFittedLine::LSFittedLine(){
	clearPoints();
}


LSFittedLine::LSFittedLine(const vector<Vector2<double> > &pointlist) {
    clearPoints();
    addPoints(pointlist);
}

LSFittedLine::~LSFittedLine(){
    points.clear();
}

void LSFittedLine::clearPoints(){
	valid = false;
	sumX = 0;
	sumY = 0;
	sumX2 = 0;
	sumY2 = 0;
    sumXY = 0;
	MSD = 0;
    r2tls = 0;
	points.clear();
}

const std::vector< Vector2<double> >& LSFittedLine::getPoints() const
{
	return points;
}

void LSFittedLine::addPoint(const Vector2<double> &point){
	sumX += point.x;
	sumY += point.y;
	sumX2 += point.x * point.x;
	sumY2 += point.y * point.y;
    sumXY += point.x * point.y;
    points.push_back(point);
    valid = points.size() >= 2;
    if(valid)
        calcLine();
}

void LSFittedLine::addPoints(const vector< Vector2<double> >& pointlist){
    if(!pointlist.empty()) {
        for(unsigned int i=0; i<pointlist.size(); i++) {
            sumX += pointlist[i].x;
            sumY += pointlist[i].y;
            sumX2 += pointlist[i].x * pointlist[i].x;
            sumY2 += pointlist[i].y * pointlist[i].y;
            sumXY += pointlist[i].x * pointlist[i].y;
            points.push_back(pointlist[i]);
        }
        valid = points.size() >= 2;
        if(valid)
            calcLine();
    }
}

void LSFittedLine::joinLine(LSFittedLine &sourceLine)
{
	sumX += sourceLine.sumX;
	sumY += sourceLine.sumY;
	sumX2 += sourceLine.sumX2;
	sumY2 += sourceLine.sumY2;
    sumXY += sourceLine.sumXY;
    for(unsigned int p = 0; p < sourceLine.points.size(); p++) {
		points.push_back(sourceLine.points[p]);
    }
    valid = points.size() >= 2;
    if(valid)
        calcLine();
}

Vector2<double> LSFittedLine::combinedR2TLSandMSD(const LSFittedLine &sourceLine) const{
    double sxx, syy, sxy, Sigma;
    double TsumX, TsumY, TsumX2, TsumY2, TsumXY, TnumPoints;
    TsumX = sumX + sourceLine.sumX;
    TsumY = sumY + sourceLine.sumY;
    TsumX2 = sumX2 + sourceLine.sumX2;
    TsumY2 = sumY2 + sourceLine.sumY2;
    TsumXY = sumXY + sourceLine.sumXY;
    TnumPoints = points.size() + sourceLine.points.size();
    Vector2<double> results;

    sxx = TsumX2 - TsumX*TsumX/TnumPoints;
    syy = TsumY2 - TsumY*TsumY/TnumPoints;
    sxy = TsumXY - TsumX*TsumY/TnumPoints;
    Sigma = (sxx+syy-sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy))/2;
    results.x = 1.0-(4.0*Sigma*Sigma/((sxx+syy)*(sxx+syy)+(sxx-syy)*(sxx-syy)+4.0*sxy*sxy));
    results.y = Sigma/TnumPoints;
    return results;
}

double LSFittedLine::getMSD () const
{
	return MSD;
}
double LSFittedLine::getr2tls () const
{
	return r2tls;
}

void LSFittedLine::calcLine(){
	double sxx, syy, sxy, Sigma;
	double A = 0, B = 0, C = 0;
    unsigned int numPoints = points.size();

    sxx = sumX2 - sumX*sumX/numPoints;
    syy = sumY2 - sumY*sumY/numPoints;
    sxy = sumXY - sumX*sumY/numPoints;
    Sigma = (sxx+syy-sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy))/2;
    //cout << "Sigma: "<< Sigma << endl;
    MSD = Sigma/numPoints;
    r2tls = 1.0-(4.0*Sigma*Sigma/((sxx+syy)*(sxx+syy)+(sxx-syy)*(sxx-syy)+4.0*sxy*sxy));


	if (sxx > syy){
		A = -sxy;
		B = (sxx-Sigma);
		C = -(sumX*sxy-(sxx-Sigma)*sumY)/numPoints;
	}
	else {
		A = (syy-Sigma);
		B = -sxy;
		C = -(sumY*sxy-(syy-Sigma)*sumX)/numPoints;
	}
	setLine(A, B, C);
}

bool LSFittedLine::getEndPoints(Vector2<double>& p1, Vector2<double>& p2) const
{
    if(points.size() < 2)
        return false;

    float min = std::numeric_limits<float>::max();
    float max = -std::numeric_limits<float>::max();
    vector< Vector2<double> >::const_iterator p, p_min, p_max;
    for(p = points.begin(), p_min = p_max = p; p!=points.end(); p++) {
        float trans_x = -m_B*p->x - m_A*p->y;
        if(trans_x < min) {
            p_min = p;
            min = trans_x;
        }
        else if(trans_x > max) {
            p_max = p;
            max = trans_x;
        }
    }
    p1 = *p_min;
    p2 = *p_max;
    return true;
}

double LSFittedLine::averageDistanceBetween(const LSFittedLine &other) const
{
    if(valid && other.valid) {
        Vector2<double> ep1, ep2, other_ep1, other_ep2;

        //no need to check this works - line is only valid if there are at least 2 points
        getEndPoints(ep1, ep2);
        other.getEndPoints(other_ep1, other_ep2);
        //project onto respective lines
        ep1 = projectOnto(ep1);
        ep2 = projectOnto(ep2);
        other_ep1 = other.projectOnto(other_ep1);
        other_ep2 = other.projectOnto(other_ep2);

        //determine distances from the two possible pairings
        double d1 = 0.5*( (ep1-other_ep1).abs() + (ep2-other_ep2).abs() ),
               d2 = 0.5*( (ep2-other_ep1).abs() + (ep1-other_ep2).abs() );
        return min(d1, d2); //best pairing results in minimum distance
    }
    return -1.0;    //test for this - distances should always be positive
}

//LinePoint::LinePoint()
//{
//    ID = 0;
//	clear();
//}

//void LinePoint::clear()
//{
//	inUse = false;
//	width = 0;
//	x = 0;
//	y = 0;
//}

//LinePoint::LinePoint(double in_x, double in_y) : Point(in_x, in_y)
//{
//    ID = 0;
//    inUse = false;
//    width = 0;
//}

/*
void test(LinePoint p1, LinePoint p2, LinePoint p3)
{
  static int testNum = 1;
  std::cout << "---Performing Test Number " << testNum << "---"<< std::endl;
  std::cout << "Point 1: (" << p1.x << "," << p1.y << ")" << std::endl;
  std::cout << "Point 2: (" << p2.x << "," << p2.y << ")" << std::endl;
  std::cout << "Point 3: (" << p3.x << "," << p3.y << ")" << std::endl;
  LSFittedLine test, copyTst;
//  bool result = test.setLineFromPoints(p1,p2);
  bool result = true;
  test.addPoint(p1);
  test.addPoint(p2);
  if(result)
  {  
    std::cout << "Line Formed: " << test.getA() << "*X + " << test.getB() << "*Y = " << test.getC() << std::endl;
    std::cout << "isValid: " << test.isValid() << std::endl;
    std::cout << "isHorizontal: " << test.isHorizontal() << std::endl;
    std::cout << "isVertical: " << test.isVertical() << std::endl;    
    std::cout << "getGradient: " << test.getGradient() << std::endl;
    std::cout << "getAngle: " << test.getAngle() * 180 / 3.14 << " Degrees." << std::endl;
    std::cout << "getXIntercept: " << test.getXIntercept() << std::endl;
    std::cout << "getYIntercept: " << test.getYIntercept() << std::endl;
    std::cout << "findYFromX("<< p1.x << "): " << test.findYFromX(p1.x) << std::endl;
    std::cout << "findYFromX("<< p2.x << "): " << test.findYFromX(p2.x) << std::endl;
    std::cout << "findXFromY(" << p1.y << "): " << test.findXFromY(p1.y) << std::endl;
    std::cout << "findXFromY(" << p2.y << "): " << test.findXFromY(p2.y) << std::endl;
    Point tst = {6,6};
    std::cout << "getLinePointDistance({6,6}): " << test.getLinePointDistance(tst) << std::endl;
    copyTst = test;
    std::cout << "Copy: " << copyTst.getA() << "*X + " << copyTst.getB() << "*Y = " << copyTst.getC() << std::endl;
    std::cout << "Copy == Original: " << (copyTst == test) << std::endl;
    std::cout << "Copy != Original: " << (copyTst != test) << std::endl;
    copyTst.setLine(copyTst.getB(),copyTst.getA(),copyTst.getC());
    std::cout << "Copy Changed: " << copyTst.getA() << "*X + " << copyTst.getB() << "*Y = " << copyTst.getC() << std::endl;
    std::cout << "Copy == Original: " << (copyTst == test) << std::endl;
    std::cout << "Copy != Original: " << (copyTst != test) << std::endl;
    std::cout << "MSD: " << test.getMSD() << std::endl;
    std::cout << "r2tls: " << test.getr2tls() << std::endl;
    
  }
  else
    std::cout << "Line Not Formed." << std::endl;
  testNum++;
  test.addPoint(p3);
  if(result)
  {  
    std::cout << "Line Formed: " << test.getA() << "*X + " << test.getB() << "*Y = " << test.getC() << std::endl;
    std::cout << "isValid: " << test.isValid() << std::endl;
    std::cout << "isHorizontal: " << test.isHorizontal() << std::endl;
    std::cout << "isVertical: " << test.isVertical() << std::endl;    
    std::cout << "getGradient: " << test.getGradient() << std::endl;
    std::cout << "getAngle: " << test.getAngle() * 180 / 3.14 << " Degrees." << std::endl;
    std::cout << "getXIntercept: " << test.getXIntercept() << std::endl;
    std::cout << "getYIntercept: " << test.getYIntercept() << std::endl;
    std::cout << "findYFromX("<< p1.x << "): " << test.findYFromX(p1.x) << std::endl;
    std::cout << "findYFromX("<< p2.x << "): " << test.findYFromX(p2.x) << std::endl;
    std::cout << "findXFromY(" << p1.y << "): " << test.findXFromY(p1.y) << std::endl;
    std::cout << "findXFromY(" << p2.y << "): " << test.findXFromY(p2.y) << std::endl;
    Point tst = {6,6};
    std::cout << "getLinePointDistance({6,6}): " << test.getLinePointDistance(tst) << std::endl;
    copyTst = test;
    std::cout << "Copy: " << copyTst.getA() << "*X + " << copyTst.getB() << "*Y = " << copyTst.getC() << std::endl;
    std::cout << "Copy == Original: " << (copyTst == test) << std::endl;
    std::cout << "Copy != Original: " << (copyTst != test) << std::endl;
    copyTst.setLine(copyTst.getB(),copyTst.getA(),copyTst.getC());
    std::cout << "Copy Changed: " << copyTst.getA() << "*X + " << copyTst.getB() << "*Y = " << copyTst.getC() << std::endl;
    std::cout << "Copy == Original: " << (copyTst == test) << std::endl;
    std::cout << "Copy != Original: " << (copyTst != test) << std::endl;
    std::cout << "MSD: " << test.getMSD() << std::endl;
    std::cout << "r2tls: " << test.getr2tls() << std::endl;
  }
}
*/
/*
int main(int argc, char* argv[])
{
  Point p1,p2,p3;
  if(argc < 6)
    std::cout << "Not enough args" << std::endl;
  for(int arg = 1; arg < argc - 5;)
  {
    p1.x = atof(argv[arg++]); p1.y = atof(argv[arg++]);
    p2.x = atof(argv[arg++]); p2.y = atof(argv[arg++]);
    p3.x = atof(argv[arg++]); p3.y = atof(argv[arg++]);
    test(p1,p2,p3);    
  }  
  return 0;
}
*/
/*
int main(int argc, char* argv[])
{
  LinePoint p1;
  LSFittedLine test, test2, test3;
  int counter = 0;
  if(argc < 4)
    std::cout << "Not enough args" << std::endl;
  for(int arg = 1; arg < argc - 1;)
  {
    p1.x = atof(argv[arg++]); p1.y = atof(argv[arg++]);
    std::cout << "Point: (" << p1.x << "," << p1.y << ")" << std::endl;
    test.addPoint(p1); 
  }
  std::cout << "Line Formed: " << test.getA() << "*X + " << test.getB() << "*Y = " << test.getC() << std::endl;
  std::cout << "isValid: " << test.isValid() << std::endl;
  std::cout << "isHorizontal: " << test.isHorizontal() << std::endl;
  std::cout << "isVertical: " << test.isVertical() << std::endl;    
  std::cout << "getGradient: " << test.getGradient() << std::endl;
  std::cout << "getAngle: " << test.getAngle() * 180 / 3.14 << " Degrees." << std::endl;
  std::cout << "getXIntercept: " << test.getXIntercept() << std::endl;
  std::cout << "getYIntercept: " << test.getYIntercept() << std::endl;
  Point tst = {6,6};
  std::cout << "getLinePointDistance({6,6}): " << test.getLinePointDistance(tst) << std::endl;
  std::cout << "MSD: " << test.getMSD() << std::endl;
  std::cout << "r2tls: " << test.getr2tls() << std::endl;

  for(counter = 1; counter < (argc - 1)/2;)
  {
    p1.x = atof(argv[counter++]); p1.y = atof(argv[counter++]);
    test2.addPoint(p1); 
  }
  for(counter = counter; counter < (argc - 1);)
  {
    p1.x = atof(argv[counter++]); p1.y = atof(argv[counter++]);
    test3.addPoint(p1); 
  }

  std::cout << "Joining: " << test3.getA() << "*X + " << test3.getB() << "*Y = " << test3.getC() << " with " << test2.getA() << "*X + " << test2.getB() << "*Y = " << test2.getC() << std::endl;
  test2.joinLine(test3);
  std::cout << "Line Formed: " << test2.getA() << "*X + " << test2.getB() << "*Y = " << test2.getC() << std::endl;
  std::cout << "isValid: " << test2.isValid() << std::endl;
  std::cout << "isHorizontal: " << test2.isHorizontal() << std::endl;
  std::cout << "isVertical: " << test2.isVertical() << std::endl;    
  std::cout << "getGradient: " << test2.getGradient() << std::endl;
  std::cout << "getAngle: " << test2.getAngle() * 180 / 3.14 << " Degrees." << std::endl;
  std::cout << "getXIntercept: " << test2.getXIntercept() << std::endl;
  std::cout << "getYIntercept: " << test2.getYIntercept() << std::endl;
  std::cout << "getLinePointDistance({6,6}): " << test2.getLinePointDistance(tst) << std::endl;
  std::cout << "MSD: " << test2.getMSD() << std::endl;
  std::cout << "r2tls: " << test2.getr2tls() << std::endl;
  return 0;
}*/
