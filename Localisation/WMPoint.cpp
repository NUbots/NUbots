#include "WMPoint.h"

WMPoint::WMPoint(double X, double Y, double Z)
{
    coordinates = Matrix(4,1,false);
    setx(X);
    sety(Y);
    setz(Z);
    coordinates[3][0] = 1;
}
WMPoint::WMPoint(Matrix point)
{
    coordinates = point;
    x = point[0][0];
    y = point[1][0];
    z = point[2][0];
}
void WMPoint::setx(double X)
{
    x = X;
    coordinates[0][0] = x;
}
void WMPoint::sety(double Y)
{
    y = Y;
    coordinates[1][0] = y;
}
void WMPoint::setz(double Z)
{
    z = Z;
    coordinates[2][0] = z;
}

