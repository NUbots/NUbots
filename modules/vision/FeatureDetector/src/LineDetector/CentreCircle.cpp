#include "CentreCircle.h"

CentreCircle::CentreCircle() {
    m_sizeOnScreen = arma::vec2(arma::zeros<vec>(2)),
    m_groundRadius = 0;
    valid = false;
    //need more here
}

CentreCircle::CentreCircle(NUPoint centre, double groundRadius, arma::vec2 screenSize) {
    m_location = centre;
    m_sizeOnScreen = screenSize,
    m_groundRadius = groundRadius;
    valid = (m_location.neckRelativeRadial[0] > 0);
}

CentreCircle::~CentreCircle() {
    // Empty destructor.
}

bool CentreCircle::addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const {
    if (valid) {
        // Add centre circle to stationary field objects.
        fieldobjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].UpdateVisualObject(m_location.neckRelativeRadial, m_sphericalError,
                                                                                                       m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                                                                                       timestamp);
    }

    return valid;
}

//! @brief Calculation of error for optimisation
double CentreCircle::findScreenError(VisionFieldObject* other) const {
    CentreCircle* c = dynamic_cast<CentreCircle*>(other);

    return (arma::norm(m_location.screenCartesian - c->m_location.screenCartesian, 2) + arma::norm(m_sizeOnScreen - c->m_sizeOnScreen, 2));
}

double CentreCircle::findGroundError(VisionFieldObject *other) const {
    CentreCircle* c = dynamic_cast<CentreCircle*>(other);

    return (arma::norm(m_location.groundCartesian - c->m_location.groundCartesian, 2) + abs(m_groundRadius - c->m_ground_radius);
}

double CentreCircle::getGroundRadius() const {
    return m_groundRadius;
}

std::ostream& operator<< (std::ostream& output, const CentreCircle& c) {
    output << "CentreCircle - " << std::endl;
    output << "\tpixelloc: " << c.m_location.screenCartesian << std::endl;
    output << "\tangularloc: " << c.m_location.screenAngular << std::endl;
    output << "\trelative field coords: " << c.m_location.neckRelativeRadial << std::endl;
    output << "\tspherical error: [" << c.m_sphericalError << "]" << std::endl;
    output << "\tsize on screen: [" << c.m_sizeOnScreen << "]";

    return output;
}

std::ostream& operator<< (std::ostream& output, const std::vector<CentreCircle>& circles) {
    for (const auto& circle : circles) {
        output << circle << std::endl;        
    }

    return output;
}
