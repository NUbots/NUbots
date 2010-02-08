#ifndef OBJECTCANDIDATE_H
#define OBJECTCANDIDATE_H

#include "Tools/Math/Vector2.h"
#include <vector>

class ObjectCandidate
{
public:
    Vector2<int> getTopLeft() const;
    Vector2<int> getBottomRight() const;

    int width();
    int height();
    float aspect();
    unsigned char getColour()  const;
    void setColour(unsigned char c);

    ObjectCandidate();
    ObjectCandidate(int left, int top, int right, int bottom);
    ObjectCandidate(int left, int top, int right, int bottom, unsigned char colour);
    ~ObjectCandidate();
protected:
    Vector2<int> topLeft;
    Vector2<int> bottomRight;
    unsigned char colour;


};

#endif // OBJECTCANDIDATE_H
