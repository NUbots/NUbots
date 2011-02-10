#include "Line.h"
#include <cmath>
#include <iostream>
#include <cstdlib>


bool operator < (const Point& point1, const Point& point2) {
    if(point1.x < point2.x) {
        return true;
    }
    else if(point1.x == point2.x) {
        if(point1.y < point2.y) {
            return true;
        }
    }
    return false;
}


// Constructor
Line::Line(){
  // General Line Equation: A*x + B*y = C
  m_A = 0.0;
  m_B = 0.0;
  m_C = 0.0;
  Rho = 0;
  Phi = 0;
  return;
}

// Constructor
Line::Line(Point p1, Point p2){
  // General Line Equation: A*x + B*y = C
  setLineFromPoints(p1,p2);
  return;
}

// Destructor
Line::~Line(){
  return;
}

// setLine(float A, float B, float C): Set the line to the given A, B, C values, if they are valid.
bool Line::setLine(double A, double B, double C)
{
  if(isValid(A,B,C) == false) // IF the values do not give a valid line, do not use them.
    return false;
  if(B != 0.0) {
      //B!=0 means not vertical
      m_A = A/B;
      m_B = 1;
      m_C = C/B;
      double denom = sqrt(m_A*m_A + m_B*m_B);
      Phi = acos(m_A/denom);
      Rho = m_C/denom;
  }
  else {
      //B==0 means vertical
      m_A = 1;
      m_B = 0.0;
      m_C = C/A;
      Phi = 0.0;
      Rho = m_C;
  }
  return (isValid()); // Just to double check.
}

// setLineFromPoints(Point p1, Point p2): Generate the line that passes through the two given points.
bool Line::setLineFromPoints(Point p1, Point p2)
{
  // Using method found at: http://www.uwm.edu/~ericskey/TANOTES/Ageometry/node4.html
  double A,B,C;
  double xDiff, yDiff;
  if( (p1.x == p2.x) && (p1.y == p2.y) ) return false; // Cannot make a line between 2 points if they are the same point.

  // go from substitution: p1.x*A + p1.y*B = p2.x*A + p2.y*B --> xDIff*A = yDiff*B (1)
  xDiff = p1.x - p2.x;
  yDiff = p2.y - p1.y;
  // Using enforced relationship A^2 + B^2 = 1 and previous result, find A.
  A = (yDiff*yDiff) / ( (yDiff*yDiff) + (xDiff*xDiff) );
  // if A = 0.0 equation is of the form y = C, so B = 1.0 stopping divide by zero. Otherwise calculate from expression (1)
  if(A == 0.0) B = 1.0;
  else B = xDiff*A / yDiff;
  // Substitute first point to find C.
  C = A*p1.x + B*p1.y;
  
  return setLine(A, B, C); // Now try to set the values.
}

bool Line::copy(const Line& source)
{
  return setLine(source.getA(), source.getB(), source.getC());
}

double Line::getA() const
{
  return m_A;
}

double Line::getB() const
{
  return m_B;
}

double Line::getC() const
{
  return m_C;
}

// isHorizontal(): Check if the current line is purely horizontal i.e. of the form y = C
bool Line::isHorizontal() const
{
  return (m_A == 0.0);
}

// isHorizontal(): Check if the current line is purely vertical i.e. of the form x = C
bool Line::isVertical() const
{
  return (m_B == 0.0);
}

// isValid(): Check if the current line is valid.
bool Line::isValid() const
{
  return isValid(m_A, m_B, m_C);
}

// findYFromX(float x): Calculate the x coord given the y for the current line.
double Line::findXFromY(double y) const
{
  double x = 1e50;
  if((isValid() == true) && (isHorizontal() == false)) // If horizontal cannot find x from y.
  {
    x = (m_C - m_B * y) / m_A; // rearrange --> x = (C - B*y) / A
  }
  return x;
}

// findYFromX(float x): Calculate the y coord given the x for the current line.
double Line::findYFromX(double x) const
{
  double y = 1e50;
  if((isValid() == true) && (isVertical() == false)) // If vertical cannot find y from x.
  {
    y = (m_C - m_A * x) / m_B; // rearrange --> y = (C - A*x) / B
  }
  return y;
}

// getGradient(): Return the gradient of the current line.
double Line::getGradient() const
{
  double gradient;
  if(isValid() == false) return 0.0;
  if(isVertical() == true)
  {
    gradient = 1e9; // Big number to represent infinity.
  }
  else
  {
    gradient = -(m_A / m_B); // rearrange equation --> y = C/B - A/B*x
  }
  return gradient;
}

double Line::getAngle() const
{
  return atan(getGradient());
}

double Line::getXIntercept() const
{
  return findXFromY(0);
}

double Line::getYIntercept() const
{
  return findYFromX(0);
}

double Line::getLinePointDistance(Point point) const
{
  double distance;
  if(isValid() == false) return 0.0;
  distance = fabs(m_A * point.x + m_B * point.y - m_C) / (sqrt(m_A*m_A + m_B*m_B));
  return distance;
}

double Line::getSignedLinePointDistance(Point point) const
{
  double distance;
  if(isValid() == false) return 0.0;
  distance = (m_A * point.x + m_B * point.y - m_C) / (sqrt(m_A*m_A + m_B*m_B));
  return distance;
}

double Line::getRho() const {
    return Rho;
}

double Line::getPhi() const {
    return Phi;
}

bool operator ==(const Line& line1, const Line& line2)
{
  return ( (line1.m_A == line2.m_A) && (line1.m_B == line2.m_B) && (line1.m_C == line2.m_C) );
}

bool operator !=(const Line& line1, const Line& line2)
{
  return !(line1 == line2);
}

bool operator >(const Line& line1, const Line& line2)
{
    if(line1.getGradient() == line2.getGradient())
        return line1.getYIntercept() > line2.getYIntercept();
    else
        return line1.getGradient() > line2.getGradient();
}

// isValid(float A, float B, float C): Check if the given values create a valid line.
bool Line::isValid(double A, double B, double C) const
{
  if((A == 0.0) && (B == 0.0)) return false; // If A = 0.0 and B = 0.0 line is not valid, as equation becomes 0.0 = C
  return true;
}
