#ifndef VISIONFIELDOBJECT_H
#define VISIONFIELDOBJECT_H

#include "Vision/basicvisiontypes.h"
#include <vector>

using namespace std;

class VisionFieldObject
{
public:
    //! VFO_ID enum and associated string conversion methods
    enum VFO_ID {
        BALL,
        GOAL_Y,
        GOAL_B,
        LINE,
        CORNER,
        CENTRE_CIRCLE,
        OBSTACLE,
        UNKNOWN
    };

    static std::string getVFOName(VFO_ID id);
    static VFO_ID getVFOFromName(const std::string& name);
    
public:
    VisionFieldObject();
    
    const PointType& getLocationPixels() const;
    const vector<float>& getLocationAngular() const;
    virtual const vector<float>& getRelativeFieldCoords() const;
    //virtual ____ convertToExternalFO() const;
    

private:
    PointType m_location_pixels;        //! @variable The pixel location of the object on the screen.
    vector<float> m_location_angular;   //! @variable The angular location of the object relative to the screen centre.
    float confidence;   //! unused
    float error;        //! unused
};

#endif // VISIONFIELDOBJECT_H
