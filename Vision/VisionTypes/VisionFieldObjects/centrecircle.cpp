#include "centrecircle.h"
#include "Vision/visionblackboard.h"
#include "debug.h"
#include "debugverbosityvision.h"

CentreCircle::CentreCircle()
{
    m_size_on_screen = Vector2<double>(0,0),
    m_ground_radius = 0;
    //need more here
}

CentreCircle::CentreCircle(GroundPoint centre, double ground_radius, Vector2<double> screen_size)
{
    m_location = centre;
    m_size_on_screen = screen_size,
    m_ground_radius =ground_radius;
    //need more here
    const Transformer& t = VisionBlackboard::getInstance()->getTransformer();

    // calculate bearing and elevation
    t.screenToRadial2D(m_location);
    valid = t.isDistanceToPointValid();
    if(valid) {
        // find distance
        double distance = t.distanceToPoint(m_location.angular.x, m_location.angular.y);
        // calculate foot relative 3D location
        t.radial2DToRadial3D(m_location, distance);
    }
}

CentreCircle::~CentreCircle()
{
}

bool CentreCircle::addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const
{
#if VISION_FIELDPOINT_VERBOSITY > 1
    debug << "CentreCircle::addToExternalFieldObjects:" << std::endl;
    debug << *this << std::endl;
#endif

    if(valid) {
        //add centre circle to stationary field objects
        fieldobjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].UpdateVisualObject(Vector3<float>(m_location.relativeRadial.x, m_location.relativeRadial.y, m_location.relativeRadial.z),
                                                                                                       Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                                       Vector2<float>(m_location.angular.x, m_location.angular.y),
                                                                                                       Vector2<int>(m_location.screen.x,m_location.screen.y),
                                                                                                       Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                                       timestamp);
    }
    return valid;
}

//! @brief Stream output for labelling purposes
void CentreCircle::printLabel(std::ostream& out) const
{
    out << m_location << " " <<  m_ground_radius << " " << m_size_on_screen;
}

//! @brief Calculation of error for optimisation
double CentreCircle::findScreenError(VisionFieldObject* other) const
{
    CentreCircle* c = dynamic_cast<CentreCircle*>(other);
    return ( m_location.screen - c->m_location.screen ).abs() + ( m_size_on_screen - c->m_size_on_screen ).abs();
}

double CentreCircle::findGroundError(VisionFieldObject *other) const
{
    CentreCircle* c = dynamic_cast<CentreCircle*>(other);
    return ( m_location.ground - c->m_location.ground ).abs() + abs( m_ground_radius - c->m_ground_radius );
}

std::ostream& operator<< (std::ostream& output, const CentreCircle& c)
{
    output << "CentreCircle - " << std::endl;
    output << "\tpixelloc: " << c.m_location.screen << std::endl;
    output << "\tangularloc: " << c.m_location.angular << std::endl;
    output << "\trelative field coords: " << c.m_location.relativeRadial << std::endl;
    output << "\tspherical error: [" << c.m_spherical_error << "]" << std::endl;
    output << "\tsize on screen: [" << c.m_size_on_screen << "]";
    return output;
}

std::ostream& operator<< (std::ostream& output, const std::vector<CentreCircle>& c)
{
    for (size_t i=0; i<c.size(); i++)
        output << c[i] << std::endl;
    return output;
}
