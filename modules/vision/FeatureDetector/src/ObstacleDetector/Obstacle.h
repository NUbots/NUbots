#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/ClassifiedImage.h"
#include "../VisionKinematics.h"
#include "../VisionFieldObject.h"

namespace modules {
    namespace vision {

        class Obstacle : public VisionFieldObject {
        public:
            Obstacle(   const VisionKinematics& visionKinematics,
                        const arma::vec2& position = arma::zeros<arma::vec>(2), 
                        double width = 0, 
                        double height = 0, 
                        messages::vision::COLOUR_CLASS colour = messages::vision::UNKNOWN_COLOUR);

            //! @brief applies a series of checks to decide if the obstacle is valid.
            bool check() const;
            
            //! @brief output stream operator.
            friend std::ostream& operator<< (std::ostream& output, const Obstacle& o);

            //! @brief output stream operator for a vector of obstacles.
            friend std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& o);
              
            friend class ObstacleDetector;
        private:
            /*!
              @brief calculates various positions values of the obstacle.
              @return whether the obstacle is valid.
              */
            bool calculatePositions(const VisionKinematics& visionKinematics);
            
            messages::vision::COLOUR_CLASS m_colour;

        //    float d2p;                      //! @variable the distance of the obstacle in cm as found by the distance to point method
            double m_arcWidth;                //! @variable the angle subtended by the obstacle (based on the screen width)
        };

    }
}
#endif // MODULES_VISION_OBSTACLE_H
