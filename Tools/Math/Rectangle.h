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
    float m_minx;
    float m_maxx;
    float m_miny;
    float m_maxy;
};

#endif // RECTANGLE_H
