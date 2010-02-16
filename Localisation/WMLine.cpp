#include "WMLine.h"
#define DTOR 0.01745329252

WMLine::WMLine(WMPoint lineStart, WMPoint lineEnd)//, bool vis)
{
    setStart(lineStart);
    setEnd(lineEnd);
}
void WMLine::setStart(WMPoint point)
{
    start = point;
}
void WMLine::setEnd(WMPoint point)
{
    end = point;
}
void WMLine::normalise()
{
    // Transform view frustrum to scaled cube between (-1,-1,-1) and (1,1,1);

    //Each WMPoint is transformed relative to its depth (z)
    double z1 = start.getz(),z2 = end.getz();

    // Create each transformation matrix dependant on z, the
    // aspect ratio (4:3) and vertical field of view (43.8)
    Matrix norm1 = Matrix(4,4,true);
    norm1[0][0] = 1 / (z1 * tan(4 * 34.8 * DTOR / 6));
    norm1[1][1] = 1 / (z1 * tan(34.8 * DTOR / 2));
    norm1[2][2] = (1.0 / 1500.0);

    Matrix norm2 = Matrix(4,4,true);
    norm2[0][0] = 1 / (z2 * tan(4 * 34.8 * DTOR / 6));
    norm2[1][1] = 1 / (z2 * tan(34.8 * DTOR / 2));
    norm2[2][2] = (1.0 / 1500.0);

    start = WMPoint(norm1 * start.get());
    end = WMPoint(norm2 * end.get());

}
void WMLine::screenCoords()
{
    // Transform from normalised coords to 2D screen coords defined
    // by a square from (0,0) to (320,240) with +x to the right and
    // +y down


    /*
      Screen Coordinate System

     O ________x            O is origin
      |
      |
      |
      y                     (z axis is dropped)

    */
    Matrix screen = Matrix(4,4,true);
    screen[0][0] = 160;
    screen[0][3] = 160;
    screen[1][1] = -120;
    screen[1][3] = 120;

    start = WMPoint(screen * start.get());
    end = WMPoint(screen * end.get());
}
bool Clip3D(WMLine& line, double near, double far)
{
        double n,f,x1,y1,z1,x2,y2,z2;
        n = near;
        f = far;
        x1 = line.getStart().getx();
        y1 = line.getStart().gety();
        z1 = line.getStart().getz();
        x2 = line.getEnd().getx();
        y2 = line.getEnd().gety();
        z2 = line.getEnd().getz();

        if((z1 < n) && (z2 < n))        // Both too close
            return false;

        if((z1 > f) && (z2 > f))        // Both too far
            return false;


        if (((z1 < n) && (z2 > n)) || ((z1 > n) && (z2 < n)))       // One too close
        {
            double ratio = (n-z1)/(z2-z1);      // ratio of amount of line in the clipping volume

            if (z1 < n)
            {
                x1 = x1 + ratio * (x2-x1);        //shorten the line by adding to the x coord
                y1 = y1 + ratio * (y2-y1);          //shorten the line by adding to the y coord
                z1 = n;                        //shorten the line by adding to the z coord

            }
            else //(z2 < n)
            {
                x2 = x1 + ratio * (x2-x1);
                y2 = y1 + ratio * (y2-y1);
                z2 = n;

            }
        }

        if (((z1 < f) && (z2 > f)) || ((z1 > f ) && (z2 < f)))      // One too far
        {
            double ratio = (f-z1)/(z2-z1);

            if (z1 < f)
            {
                x2 = x1 + ratio * (x2-x1);
                y2 = y1 + ratio * (y2-y1);
                z2 = f;
            }
            else //(z1 > f)
            {
                x1 = x1 + ratio * (x2-x1);
                y1 = y1 + ratio * (y2-y1);
                z1 = f;
            }
        }

        WMPoint newStart = WMPoint(x1,y1,z1);
        WMPoint newEnd = WMPoint(x2,y2,z2);

        line = WMLine(newStart,newEnd);
        return true;
}
bool Clip2D(WMLine& line)
{
        double x1,y1,z1,x2,y2,z2;

        x1 = line.getStart().getx();
        y1 = line.getStart().gety();
        z1 = line.getStart().getz();

        x2 = line.getEnd().getx();
        y2 = line.getEnd().gety();
        z2 = line.getEnd().getz();

        if ((x1 > 1) && (x2 > 1))       // Completely right of x = 1
            return false;

        if ((x1 < -1) && (x2 < -1))     // Completely left of x = -1
            return false;


        if (((x1 > 1) && (x2 < 1)) || ((x1 < 1) && (x2 > 1)))      // Crosses x = 1
        {
            double ratio = (1 - x1) / (x2 - x1);
            if (x1 < 1)
            {
                y2 = y1 + ratio * (y2-y1);
                x2 = 1;
            }
            else // (x2 < 1)
            {
                y1 = y1 + ratio * (y2-y1);
                x1 = 1;
            }
        }
        if ((x1 < -1 && x2 > -1) || (x1 > -1 && x2 < -1))      // Crosses x = -1
        {
            double ratio = (-1 - x1) / (x2 - x1);
            if (x1 > -1)
            {
                y2 = y1 + ratio * (y2-y1);
                x2 = -1;
            }
            else // (x2 < 1)
            {
                y1 = y1 + ratio * (y2-y1);
                x1 = -1;
            }
        }

        if (y1 > 1 && y2 > 1)           // Completely above y = 1
            return false;

        if (y1 < -1 && y2 < -1)         // Completely below y = -1
            return false;

        if ((y1 > 1 && y2 < 1) || (y1 < 1 && y2 > 1))       //Crosses y = 1
        {
            double ratio = (1 - y1) / (y2 - y1);
            if (y1 < 1)
            {
                x2 = x1 + ratio * (x2 - x1);
                y2 = 1;
            }
            else //(y2 < 1)
            {
                x1 = x1 + ratio * (x2-x1);
                y1 = 1;
            }
        }
        if ((y1 < -1 && y2 > -1) || (y1 > -1 && y2 < -1))
        {
            double ratio = (-1 - y1) / (y2 - y1);
            if (y1 > -1)
            {
                x2 = x1 + ratio * (x2 - x1);
                y2 = -1;
            }
            else //(y2 > -1)
            {
                x1 = x1 + ratio * (x2-x1);
                y1 = -1;
            }
        }

        if (x1 > 1)
            x1 = 1;
        else if (x1 < -1)
            x1 = -1;
        if (x2 > 1)
            x2 = 1;
        else if (x2 < -1)
            x2 = -1;

        if (y1 > 1)
            y1 = 1;
        else if (y1 < -1)
            y1 = -1;
        if (y2 > 1)
            y2 = 1;
        else if (y2 < -1)
            y2 = -1;

        WMPoint newStart = WMPoint(x1,y1,z1);
        WMPoint newEnd = WMPoint(x2,y2,z2);

        line = WMLine(newStart,newEnd);
        return true;
}
