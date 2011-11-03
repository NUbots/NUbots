#include "LocalisationPerformanceMeasure.h"

LocalisationPerformanceMeasure::LocalisationPerformanceMeasure()
{
    m_processing_time = 0.0f;
}

LocalisationPerformanceMeasure::LocalisationPerformanceMeasure(const LocalisationPerformanceMeasure& source)
{
    m_processing_time = source.m_processing_time;
    m_err_x = source.m_err_x;
    m_err_y = source.m_err_y;
    m_err_heading = source.m_err_heading;
}
