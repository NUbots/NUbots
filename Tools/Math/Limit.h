/*!
  @file Limit.h
  @author Steven Nicklin
  @brief Class used to specify a limit.
*/

#pragma once
#include <algorithm>
// Class used to limit a value.
class Limit
{
public:
    Limit(): m_min(0.0f), m_max(0.0f){}
    Limit(float min, float max): m_min(min), m_max(max){}

    void setLimits(float min, float max)
    {
        m_min = min;
        m_max = max;
        return;
    }

    float min() const
    {
        return m_min;
    }

    float max() const
    {
        return m_max;
    }

    float clip(float value) const
    {
        float clippedValue = std::max(value, min());
        clippedValue = std::min(clippedValue, max());
        return clippedValue;
    }

protected:
    float m_min;
    float m_max;
};

