#include "VisionFieldObject.h"

VisionFieldObject::VisionFieldObject() {
    // Empty constructor.
}

virtual VisionFieldObject::~VisionFieldObject() {
    // Empty destructor.
}

VFO_ID VisionFieldObject::getID() const {
    return m_id;
}

std::string VisionFieldObject::getName() const	{
    return VFOName(m_id);
}

bool VisionFieldObject::isValid() const {
    return valid;
}

const NUPoint& VisionFieldObject::getLocation() const {
    return m_location;
}

arma::vec2 VisionFieldObject::getScreenSize() const {
    return m_sizeOnScreen;
}

virtual arma::vec3 VisionFieldObject::getRelativeFieldCoords() const {
    return m_location.neckRelativeRadial;
}

arma::vec2 VisionFieldObject::getLocationPixels() const {
    return m_location.screenCartesian;
}

arma::vec2 VisionFieldObject::getLocationAngular() const {
    return m_location.screenAngular;
}
