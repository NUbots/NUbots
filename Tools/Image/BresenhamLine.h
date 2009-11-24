/*!
@file BrensenhamLine.h
@brief Declaration of BrensenhamLine class.
@author Steven Nicklin
*/

#ifndef BRESENHAMLINE_H
#define BRESENHAMLINE_H

#include "Tools/Math/Vector2.h"

// http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm


class BresenhamLine
{
public:
    BresenhamLine(const Vector2<int>& start, const Vector2<int>& end);
    void increment(Vector2<int>& point);
    void reset()
    {
        error = initialError;
    };

private:
    int error;
    int errorDelta;
    int initialError;
    int resetError;

    Vector2<int> standardIncrement;
    Vector2<int> errorIncrement;

    void configure(const Vector2<int>& direction);
};

#endif // BRESENHAMLINE_H
