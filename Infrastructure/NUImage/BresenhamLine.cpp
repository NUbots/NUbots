#include "BresenhamLine.h"
#include "Tools/Math/General.h"


BresenhamLine::BresenhamLine(const Vector2<int>& start, const Vector2<int>& end)
{
    configure(end - start);
    return;
}

void BresenhamLine::increment(Vector2<int>& point)
{
    point += standardIncrement;
    error += errorDelta;
    if(error < 0)
    {
        point += errorIncrement;
        error += resetError;
    }
    return;
}

void BresenhamLine::configure(const Vector2<int>& direction)
{
    int dx = direction.x;
    int dy = direction.y;
    int absDx = mathGeneral::abs(dx);
    int absDy = mathGeneral::abs(dy);
    bool steep = (absDy > absDx);

    // Increment values
    int incX = ((dx>0) ? 1:-1);
    int incY = ((dy>0) ? 1:-1);

    if(steep)
    {
        initialError = absDy;
        errorDelta = -2*absDx;
        standardIncrement.x = 0;
        standardIncrement.y = incY;
        errorIncrement.x = incX;
        errorIncrement.y = 0;
    }
    else
    {
        initialError = absDx;
        errorDelta = -2*absDy;
        standardIncrement.x = incX;
        standardIncrement.y = 0;
        errorIncrement.x = 0;
        errorIncrement.y = incY;
    }
    resetError = 2*initialError;
    error = initialError;
}
