#ifndef LOCALISATIONPERFORMANCEMEASURE_H
#define LOCALISATIONPERFORMANCEMEASURE_H

class LocalisationPerformanceMeasure
{
public:
    LocalisationPerformanceMeasure();
    LocalisationPerformanceMeasure(const LocalisationPerformanceMeasure& source);
    float processingTime() {return m_processing_time;}
    void setProcessingTime(float new_time){m_processing_time = new_time;}
    void setError(float x, float y, float heading)
    {
        m_err_x = x;
        m_err_y = y;
        m_err_heading = heading;
    }
    float error_x() const {return m_err_x;}
    float error_y() const {return m_err_y;}
    float error_heading() const {return m_err_heading;}

protected:
    float m_processing_time;
    float m_err_x, m_err_y, m_err_heading;

};

#endif // LOCALISATIONPERFORMANCEMEASURE_H
