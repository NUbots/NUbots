#ifndef MODULES_VISION_FIELDPOINTDETECTOR_H
#define MODULES_VISION_FIELDPOINTDETECTOR_H

#include <vector>
#include <armadillo>

#include "messages/vision/ClassifiedImage.h"

#include "../VisionKinematics.h"
#include "../NUPoint.h"
// #include "CircleDetector.h"
// #include "LineDetector.h"
// #include "CornerDetector.h"

namespace modules {
    namespace vision {

        class FieldPointDetector {
        public:
            FieldPointDetector(/*LineDetector* lineDetector = NULL, CircleDetector* circleDetector = NULL, CornerDetector* cornerDetector = NULL*/);

            void setParameters(bool TRANSFORM_FIRST_);

            void run(bool findCircles,
                    bool findLines, 
                    bool findCorners, 
                    const VisionKinematics& transformer, 
                    const std::vector<arma::vec2>& greenHorizon) const;

//            LineDetector* m_lineDetector;
//            CircleDetector* m_circleDetector;
//            CornerDetector* m_cornerDetector;
            
        private:
            bool TRANSFORM_FIRST;
        };
    }
}

#endif // MODULES_VISION_FIELDPOINTDETECTOR_H
