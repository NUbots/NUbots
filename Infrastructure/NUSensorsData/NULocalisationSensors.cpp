#include "NULocalisationSensors.h"

NULocalisationSensors::NULocalisationSensors(): m_gps(""), m_compass(""), m_odometry(""), m_falling(""),
                                                m_fallen(""), m_left_foot(""), m_right_foot("")
{
    m_time = 0.0;
}

NULocalisationSensors::NULocalisationSensors(double time, const Sensor& gps, const Sensor& compass, const Sensor& odom,
                                             const Sensor& falling, const Sensor& fallen,const Sensor& lf, const Sensor& rf):
                                            m_gps(gps), m_compass(compass), m_odometry(odom), m_falling(falling), m_fallen(fallen),
                                            m_left_foot(lf), m_right_foot(rf)
{
    m_time = time;
}

ostream& operator<< (ostream& output, const NULocalisationSensors& p_sensor)
{
    output << p_sensor.gps();
    output << p_sensor.compass();
    output << p_sensor.odometry();
    output << p_sensor.falling();
    output << p_sensor.fallen();
    output << p_sensor.leftFoot();
    output << p_sensor.rightFoot();
    return output;
}

istream& operator>> (istream& input, NULocalisationSensors& p_sensor)
{
    input >> p_sensor.m_gps;
    input >> p_sensor.m_compass;
    input >> p_sensor.m_odometry;
    input >> p_sensor.m_falling;
    input >> p_sensor.m_fallen;
    input >> p_sensor.m_left_foot;
    input >> p_sensor.m_right_foot;
    p_sensor.m_time = p_sensor.m_odometry.Time;
}
