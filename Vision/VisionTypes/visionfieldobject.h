#ifndef VISIONFIELDOBJECT_H
#define VISIONFIELDOBJECT_H

#include "basicvisiontypes.h"


class VisionFieldObject
{
public:
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

    /*! @var map id_map
     *  @brief Static map between internal and external FO IDs.
     */
    static std::map<VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> id_map;

    static std::string getVFOName(VFO_ID id);
    static VFO_ID getVFOFromName(const std::string& name);
    
public:
    VisionFieldObject();
    VisionFieldObject(VFO_ID id);

private:
    VFO_ID id;
};

#endif // VISIONFIELDOBJECT_H
