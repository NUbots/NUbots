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

CentreCircle::CentreCircle(NUPoint centre, double ground_radius, Vector2<double> screen_size)
{
    m_location = centre;
    m_size_on_screen = screen_size,
    m_ground_radius =ground_radius;
    //need more here
    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();

    tran.calculateRepresentations(m_location);
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
        fieldobjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                                                                                       Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                                       Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                                                                                       Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                                                                                       Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                                       timestamp);
    }
    return valid;
}

//! @brief Stream output for labelling purposes
void CentreCircle::printLabel(std::ostream& out) const
{
    out << VFOName(CENTRE_CIRCLE) << " " << m_location << " " <<  m_ground_radius << " " << m_size_on_screen;
}

//! @brief Calculation of error for optimisation
double CentreCircle::findScreenError(VisionFieldObject* other) const
{
    CentreCircle* c = dynamic_cast<CentreCircle*>(other);
    return ( m_location.screenCartesian - c->m_location.screenCartesian ).abs() + ( m_size_on_screen - c->m_size_on_screen ).abs();
}

double CentreCircle::findGroundError(VisionFieldObject *other) const
{
    CentreCircle* c = dynamic_cast<CentreCircle*>(other);
    return ( m_location.groundCartesian - c->m_location.groundCartesian ).abs() + abs( m_ground_radius - c->m_ground_radius );
}

std::ostream& operator<< (std::ostream& output, const CentreCircle& c)
{
    output << "CentreCircle - " << std::endl;
    output << "\tpixelloc: " << c.m_location.screenCartesian << std::endl;
    output << "\tangularloc: " << c.m_location.screenAngular << std::endl;
    output << "\trelative field coords: " << c.m_location.neckRelativeRadial << std::endl;
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
