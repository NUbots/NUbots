#ifndef BASICVISIONTYPES_H
#define BASICVISIONTYPES_H

#include <opencv2/core/core.hpp>

typedef cv::Point2i PointType;      //! Use opencv points.
//typedef cv::Scalar Quad;

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
