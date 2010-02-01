#include "ObjectCandidate.h"

ObjectCandidate::ObjectCandidate()
{
    topLeft.x = 0;
    topLeft.y = 0;
    bottomRight.x = 0;
    bottomRight.y = 0;
}

ObjectCandidate::ObjectCandidate(int left, int top, int right, int bottom)
{
    topLeft.x = left;
    topLeft.y = top;
    bottomRight.x = right;
    bottomRight.y = bottom;
}

ObjectCandidate::~ObjectCandidate()
{
    return;
}

Vector2<int> ObjectCandidate::getTopLeft() const
{
    return topLeft;
}
Vector2<int> ObjectCandidate::getBottomRight() const
{
    return bottomRight;
}

int ObjectCandidate::width()
{
    return (bottomRight.x - topLeft.x);
}
int ObjectCandidate::height()
{
    return (bottomRight.y - topLeft.y);
}
float ObjectCandidate::aspect()
{
    return (float)(bottomRight.x - topLeft.x) / (float)(bottomRight.y - topLeft.y);
}

