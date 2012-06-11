#include "MeasurementError.h"

MeasurementError::MeasurementError(): m_distance(1.0f), m_heading(1.0f)
{
}

MeasurementError::MeasurementError(float distance_error, float heading_error): m_distance(distance_error), m_heading(heading_error)
{

}
