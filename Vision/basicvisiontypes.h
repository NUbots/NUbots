#ifndef BASICVISIONTYPES_H
#define BASICVISIONTYPES_H

#include <opencv2/core/core.hpp>
#include "Tools/Math/Vector2.h"

typedef Vector2<double> Point;

namespace VisionID {
    enum ScanDirection {
        VERTICAL,
        HORIZONTAL
    };

//    enum EXTERNAL_FIELD_OBJECT_ID {
//        BALL,
//        GOAL_Y,
//        GOAL_B,
//        LINE,
//        CORNER,
//        CENTRE_CIRCLE,
//        OBSTACLE,
//        UNKNOWN
//    };
}



#endif // BASICVISIONTYPES_H
