#ifndef RECTANGLE_H
#define RECTANGLE_H

class Rectangle
{
public:
    Rectangle();
    Rectangle(const Rectangle& source);
    Rectangle(float minx, float maxx, float miny, float maxy);
    void Set(float minx, float maxx, float miny, float maxy);
    float Width();
    float Height();
    float Area();

    float CentreX();
    float CentreY();

    float MinX();
    float MaxX();
    float MinY();
    float MaxY();

private:
    float minx;
    float maxx;
    float miny;
    float maxy;
};

#endif // RECTANGLE_H
