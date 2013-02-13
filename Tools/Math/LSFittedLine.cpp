#include "LSFittedLine.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
using namespace std;

LSFittedLine::LSFittedLine(){
	clearPoints();
}

void LSFittedLine::clearPoints(){
	valid = false;
	sumX = 0;
	sumY = 0;
	sumX2 = 0;
	sumY2 = 0;
	sumXY = 0;
    numPoints = 0;
	leftPoint.x = 0;
	leftPoint.y = 0;
	rightPoint.x = 0;
    rightPoint.y = 0;
	MSD = 0;
    r2tls = 0;
	points.clear();
}

const std::vector<LinePoint>& LSFittedLine::getPoints()
{
	return points;
}

void LSFittedLine::addPoint(LinePoint &point){
	sumX += point.x;
	sumY += point.y;
	sumX2 += point.x * point.x;
	sumY2 += point.y * point.y;
	sumXY += point.x * point.y;
	numPoints ++;
    points.push_back(point);
	point.inUse = true;
	if (numPoints < 2)
	{
		valid = false;
		leftPoint = point;
		rightPoint = point;
	}
	else
	{
		valid = true;
		calcLine();
		//CHECK if new point is a start or end point
		if (point.x < leftPoint.x)
			leftPoint = point;
		else if (point.x > rightPoint.x)
			rightPoint = point;
		
		//SPECIAL CONDITION FOR VERITCAL LINES
		//**************************************
		else if (rightPoint.x == leftPoint.x)
		{
			if(point.y < leftPoint.y)
				leftPoint = point;
			else if(point.y > rightPoint.y)
				rightPoint = point;
		}
	}
}

void LSFittedLine::addPoints(vector<LinePoint>& pointlist){
    if(!pointlist.empty()) {
        if(numPoints < 1) {
            leftPoint = pointlist[0];
            rightPoint = pointlist[0];
        }
        for(unsigned int i=0; i<pointlist.size(); i++) {
            sumX += pointlist[i].x;
            sumY += pointlist[i].y;
            sumX2 += pointlist[i].x * pointlist[i].x;
            sumY2 += pointlist[i].y * pointlist[i].y;
            sumXY += pointlist[i].x * pointlist[i].y;
            numPoints++;
            points.push_back(pointlist[i]);
            pointlist[i].inUse = true;

            //CHECK if point is a start or end point
            if(pointlist[i].x == leftPoint.x){
                if(pointlist[i].y < leftPoint.y){
                    leftPoint = pointlist[i];
                }
            }
            else if (pointlist[i].x < leftPoint.x) {
                leftPoint = pointlist[i];
            }
            if(pointlist[i].x == rightPoint.x) {
                if(pointlist[i].y > rightPoint.y) {
                    rightPoint = pointlist[i];
                }
            }
            else if (pointlist[i].x > rightPoint.x) {
                rightPoint = pointlist[i];
            }
        }
        if (numPoints < 2)
        {
                valid = false;
        }
        else
        {
                valid = true;
                calcLine();
        }
    }
}

void LSFittedLine::joinLine(LSFittedLine &sourceLine)
{
	sumX += sourceLine.sumX;
	sumY += sourceLine.sumY;
	sumX2 += sourceLine.sumX2;
	sumY2 += sourceLine.sumY2;
	sumXY += sourceLine.sumXY;
	numPoints += sourceLine.numPoints;
    for(unsigned int p = 0; p < sourceLine.points.size(); p++) {
		points.push_back(sourceLine.points[p]);
	}
    if (numPoints < 2 && sourceLine.numPoints > 0) {
		valid = false;		
		leftPoint = sourceLine.leftPoint;
		rightPoint = sourceLine.rightPoint;
	}
    else {
		valid = true;
		calcLine();
		//CHECK if new point is a start or end point
		if (sourceLine.leftPoint.x < leftPoint.x)
			leftPoint = sourceLine.leftPoint;
        if (sourceLine.rightPoint.x > rightPoint.x)
			rightPoint = sourceLine.rightPoint;
		
		//SPECIAL CONDITION FOR VERITCAL LINES
		//**************************************
        if (rightPoint.x == leftPoint.x) {
			if(sourceLine.leftPoint.y < leftPoint.y)
				leftPoint = sourceLine.leftPoint;
            if(sourceLine.rightPoint.y > rightPoint.y)
				rightPoint = sourceLine.rightPoint;
		}
	}
}

Vector2<double> LSFittedLine::combinedR2TLSandMSD(const LSFittedLine &sourceLine) const{
    double sxx, syy, sxy, Sigma;
    double TsumX, TsumY, TsumX2, TsumY2, TsumXY, TnumPoints;
    TsumX = sumX + sourceLine.sumX;
    TsumY = sumY + sourceLine.sumY;
    TsumX2 = sumX2 + sourceLine.sumX2;
    TsumY2 = sumY2 + sourceLine.sumY2;
    TsumXY = sumXY + sourceLine.sumXY;
    TnumPoints = numPoints + sourceLine.numPoints;
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

LinePoint::LinePoint()
{
    ID = 0;
	clear();
}

void LinePoint::clear()
{
	inUse = false;
	width = 0;
	x = 0;
	y = 0;
}

LinePoint::LinePoint(double in_x, double in_y) : Point(in_x, in_y)
{
    ID = 0;
    inUse = false;
    width = 0;
}

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
