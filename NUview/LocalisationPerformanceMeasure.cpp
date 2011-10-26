#include "LocalisationPerformanceMeasure.h"

LocalisationPerformanceMeasure::LocalisationPerformanceMeasure()
{
    m_processing_time = 0.0f;
}

LocalisationPerformanceMeasure::LocalisationPerformanceMeasure(const LocalisationPerformanceMeasure& source)
{
    m_processing_time = source.m_processing_time;
}
