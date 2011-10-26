#ifndef LOCALISATIONPERFORMANCEMEASURE_H
#define LOCALISATIONPERFORMANCEMEASURE_H

class LocalisationPerformanceMeasure
{
public:
    LocalisationPerformanceMeasure();
    LocalisationPerformanceMeasure(const LocalisationPerformanceMeasure& source);
    float processingTime() {return m_processing_time;}
    void setProcessingTime(float new_time){m_processing_time = new_time;}
protected:
    float m_processing_time;

};

#endif // LOCALISATIONPERFORMANCEMEASURE_H
