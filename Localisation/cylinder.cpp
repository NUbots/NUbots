#include "cylinder.h"

Cylinder::Cylinder(WMPoint start, WMPoint end)
{
    left = WMLine();
    right = WMLine();
    mid = WMLine(start,end);
}
void Cylinder::setMid(WMPoint start, WMPoint end)
{
    mid = WMLine(start,end);
}
void Cylinder::setLeft(WMPoint start, WMPoint end)
{
    left = WMLine(start,end);
}
void Cylinder::setRight(WMPoint start, WMPoint end)
{
    right = WMLine(start,end);
}
void Cylinder::createSides()
{
    double x1 = mid.getStart().getx();
    double x2 = mid.getEnd().getx();
    double y1 = mid.getStart().gety();
    double y2 = mid.getEnd().gety();
    double z1 = mid.getStart().getz();
    double z2 = mid.getEnd().getz();


    double leftXTop,leftYTop,leftXBase,leftYBase,rightXTop,rightYTop,rightXBase,rightYBase;

    double angle = (atan((x1-x2) / (y1-y2)));

    leftXTop = x2 - 5 * cos(angle);
    rightXTop = x2 + 5 * cos(angle);

    leftYTop = y2 + 5 * sin(angle);
    rightYTop = y2 - 5 * sin(angle);

    leftXBase = x1 - 5 * cos(angle);
    rightXBase = x1 + 5 * cos(angle);

    leftYBase = y1 + 5 * sin(angle);
    rightYBase = y1 - 5 * sin(angle);

    left = WMLine(WMPoint(leftXBase,leftYBase,z1),WMPoint(leftXTop,leftYTop,z2));
    right = WMLine(WMPoint(rightXBase,rightYBase,z1),WMPoint(rightXTop,rightYTop,z2));

}
