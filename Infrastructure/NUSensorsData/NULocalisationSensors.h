#ifndef NULOCALISATIONSENSORS_H
#define NULOCALISATIONSENSORS_H
#include "Sensor.h"
#include "Tools/FileFormats/TimestampedData.h"
class NULocalisationSensors: public TimestampedData
{
public:
    NULocalisationSensors();
    NULocalisationSensors(double time, const Sensor& gps, const Sensor& compass, const Sensor& odom,
                          const Sensor& falling, const Sensor& fallen, const Sensor& getup, const Sensor& lf, const Sensor& rf);
    Sensor gps() const {return m_gps;}
    Sensor compass() const {return m_compass;}
    Sensor odometry() const {return m_odometry;}
    Sensor falling() const {return m_falling;}
    Sensor fallen() const {return m_fallen;}
    Sensor getup() const {return m_getup;}
    Sensor leftFoot() const {return m_left_foot;}
    Sensor rightFoot() const {return m_right_foot;}
    double GetTimestamp() const {return m_time;}

    friend std::ostream& operator<< (std::ostream& output, const NULocalisationSensors& p_sensor);
    friend std::istream& operator>> (std::istream& input, NULocalisationSensors& p_sensor);
protected:
    Sensor m_gps;                   //!< gps sensor
    Sensor m_compass;               //!< compass sensor
    Sensor m_odometry;              //!< odometry sensor
    Sensor m_falling;         //!< incapacitated sensor
    Sensor m_fallen;
    Sensor m_getup;
    Sensor m_left_foot;
    Sensor m_right_foot;
    double m_time;

};

#endif // NULOCALISATIONSENSORS_H
