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

bool VisionFieldObject::isGoal(VFO_ID id) const {
    return ((id >= GOAL_L) && (id <= GOAL_U);
}

std::string VisionFieldObject::VFOName(VFO_ID id)
{
    std::map<VFO_ID, std::pair<int, std::string> >::const_iterator it = vfomap.find(id);

    if(it == vfomap.end()) {
        throw "Invalid VFO_ID passed to VFOName()";
    }
    else {
        return it->second.second;
    }
}

VFO_ID VisionFieldObject::VFOFromName(const std::string &name)
{
    std::map<VFO_ID, std::pair<int, std::string> >::const_iterator it = vfomap.begin();

    while(it != vfomap.end() && it->second.second.compare(name) != 0)
        it++;

    if(it == vfomap.end()) {
        throw "Invalid VFO_ID name passed to VFOFromName()";
    }
    else {
        return it->first;
    }
}

VFO_ID VisionFieldObject::VFOFromInt(int n) {
    std::map<VFO_ID, std::pair<int, std::string> >::const_iterator it = vfomap.begin();

    while(it != vfomap.end() && it->second.first != n)
        it++;

    if(it == vfomap.end()) {
        throw "Invalid VFO_ID number passed to VFOFromInt()";
    }
    else {
        return it->first;
    }
}

int VisionFieldObject::intFromVFO(VFO_ID id) {
    std::map<VFO_ID, std::pair<int, std::string> >::const_iterator it = vfomap.find(id);

    if(it == vfomap.end()) {
        throw "Invalid VFO_ID passed to intFromVFO()";
    }
    else {
        return it->second.first;
    }
}

int VisionFieldObject::numVFOIDs()
{
    return vfomap.size();
}

DistanceMethod VisionFieldObject::getDistanceMethodFromName(const std::string& name) {
    if (name.compare("WIDTH") == 0) {
        return Width;
    }

    else if (name.compare("D2P") == 0) {
        return D2P;
    }

    else if (name.compare("LEAST") == 0) {
        return Least;
    }

    else if (name.compare("AVERAGE") == 0) {
        return Average;
    }

    else {
        return D2P;
    }
}

std::string VisionFieldObject::getDistanceMethodName(const DistanceMethod& method) {
    switch (method) {
        case Width: {
            return "WIDTH";
        }

        case D2P: {
            return "D2P";
        }

        case Average: {
            return "AVERAGE";
        }

        case Least: {
            return "LEAST";
        }

        default: {
            return "UNKOWN";
        }
    }
}
