#include "sphere.h"


Sphere::Sphere(double x, double y)
{
    centre = WMPoint(x,y,4);
    top = WMLine();
    bottom = WMLine();
}
void Sphere::setCentre(WMPoint newCentre)
{
    centre = WMPoint(newCentre);}

void Sphere::createSides()
{
    double x = centre.getx();
    double y = centre.gety();
    double z = centre.getz();

    WMPoint topLeft = WMPoint(x-3.25,y+3.25,z);
    WMPoint topRight = WMPoint(x+3.25,y+3.25,z);
    WMPoint bottomLeft = WMPoint(x-3.25,y-3.25,z);
    WMPoint bottomRight = WMPoint(x+3.25,y-3.25,z);

    top = WMLine(topLeft,topRight);
    bottom = WMLine(bottomLeft,bottomRight);
    mid = WMLine(centre,centre);
}
bool Clip3D(Sphere sphere,double near, double far)
{
    double z = sphere.getCentre().getz();
    return ((z >= (near-3.25)) && (z<=(far + 3.25)));
}
