#include "LocWmFrame.h"
#include "Localisation.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

LocWmFrame::LocWmFrame(): m_buffered(true)
{
    m_loc = new Localisation();
    m_sensors = new NUSensorsData();
    m_objects = new FieldObjects();
}

LocWmFrame::LocWmFrame(Localisation* loc, NUSensorsData* sensors, FieldObjects* objects):
        m_buffered(false),
        m_loc(loc),
        m_sensors(sensors),
        m_objects(objects)
{
}

LocWmFrame::~LocWmFrame()
{
    if(m_buffered)
    {
        if(m_loc) delete m_loc;
        if(m_sensors) delete m_sensors;
        if(m_objects) delete m_objects;
    }
}

double LocWmFrame::GetTimestamp() const
{
    if(m_sensors) return m_sensors->GetTimestamp();
    else return 0.0;
}

std::ostream& operator<< (std::ostream& output, const LocWmFrame& p_loc)
{
    output << *(p_loc.m_loc);
    output << *(p_loc.m_sensors);
    output << *(p_loc.m_objects);
    return output;
}

std::istream& operator>> (std::istream& input, LocWmFrame& p_loc)
{
    input >> *(p_loc.m_loc);
    input >> *(p_loc.m_sensors);
    input >> *(p_loc.m_objects);
    return input;
}
