#ifndef QUAD_H
#define QUAD_H

#include <opencv2/core/core.hpp>
#include "Tools/Math/Vector2.h"

#include "Vision/basicvisiontypes.h"

class Quad
{
public:
    
    Quad();
    Quad(const Quad& other);
    Quad(int left, int top, int right, int bottom);
    
    Vector2<int> getBottomCentre() const;
    Vector2<int> getCentre() const;
    PointType getBottomLeft() const;
    PointType getTopRight() const;
    int getWidth() const;
    int getHeight() const;
    
    cv::Scalar getAsScalar() const;
    
private:
    int m_left, m_right, m_top, m_bottom;
    cv::Scalar vals;
};

#endif // QUAD_H
