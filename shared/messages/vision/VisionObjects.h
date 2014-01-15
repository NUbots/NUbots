
#ifdef MESSAGES_VISION_VISION_OBJECTS_H
#define MESSAGES_VISION_VISION_OBJECTS_H

#include <armadillo> 
#include <nuclear>

namespace messages{
    namespace vision {

        class VisionObject {
            arma::vec3 sphericalFromNeck;    //neckRelativeRadial
            arma::vec3 sphericalError;
            arma::vec2 screenAngular;    //Polar around view vector on image
            arma::vec2 screenCartesian;
            arma::vec2 sizeOnScreen;
            NUClear::clock::time_point timestamp;
        };

        class Ball : VisionObject {
            Ball() : VisionObject() {}            
        };

        class Goal : VisionObject {
            Goal() : VisionObject() {}
            enum Type{
                LEFT,
                RIGHT,
                UNKOWN
            } type;
        };

        class Obstacle : VisionObject {
            Obstacle() : VisionObject() {}
            float arcWidth;
        };    




        //Line objects:

        class FieldLine : VisionObject {
            FieldLine() : VisionObject() {}
        };

        class CornerPoint : VisionObject {
            CornerPoint() : VisionObject() {}
            enum Type{
                L_CORNER,
                T_CORNER,
                X_CORNER
            } type;
        };

        class CentreCircle : VisionObject {
            CentreCircle() : VisionObject() {}        
        };

        class LineObjects{
            LineObjects(){}
            std::vector<CentreCircle> centre_circles;
            std::vector<CornerPoint> corner_points;
            std::vector<FieldLine> field_lines;
        };

    }
}

#endif