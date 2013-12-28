/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.cpp
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#include "ObstacleDetector.h"

namespace modules {
    namespace vision {

        using messages::vision::ColourSegment;
        using messages::vision::COLOUR_CLASS;

        ObstacleDetector::ObstacleDetector() {
            // Empty constructor.
        }

        void ObstacleDetector::setParameters(int MIN_DISTANCE_FROM_HORIZON_,
                                             unsigned int VERTICAL_SCANLINE_SPACING_,
                                             int MIN_CONSECUTIVE_POINTS_,
                                             int MIN_COLOUR_THRESHOLD_,
                                             int MAX_OTHER_COLOUR_THRESHOLD_,
                                             int VER_THRESHOLD_,
                                             double OBJECT_THRESHOLD_MULT_) {
            MIN_DISTANCE_FROM_HORIZON = MIN_DISTANCE_FROM_HORIZON_;
            VERTICAL_SCANLINE_SPACING = VERTICAL_SCANLINE_SPACING_;
            MIN_CONSECUTIVE_POINTS = MIN_CONSECUTIVE_POINTS_;

            MIN_COLOUR_THRESHOLD = MIN_COLOUR_THRESHOLD_;
            MAX_OTHER_COLOUR_THRESHOLD = MAX_OTHER_COLOUR_THRESHOLD_;

            VER_THREHOLD = VER_THRESHOLD_;
            OBJECT_THRESHOLD_MULT = OBJECT_THRESHOLD_MULT_;
        }

        std::vector<Obstacle> ObstacleDetector::run(const std::vector<arma::vec2>& greenHorizon) {
            // get blackboard instance
//            const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
//            const NUImage& img = vbb->getOriginalImage();

            std::vector<arma::vec2> horizonPoints = greenHorizon;
            std::vector<arma::vec2> obstaclePoints;
            std::vector<Obstacle> obstacles;

            int height = 10; // TODO: img.getHeight();
            double mean_y, std_dev_y;

            // Get scan points from BB.
            // TODO: Add functionality to obtain a subset of the green horizon.
            // TODO: Do we need to include the entire GreenHorizon object in the classified image?
//            horizon_points = green_horizon.getInterpolatedSubset(VisionConstants::VERTICAL_SCANLINE_SPACING);

            // Calculate mean and stddev of vertical positions.
            arma::running_stat<double> acc;

            accumulator_set<double, stats<tag::mean, tag::variance> > acc;

            BOOST_FOREACH(const arma::vec2& point : horizonPoints) {
                acc(point[1]);
            }

            mean_y = acc.mean();
            std_dev_y = acc.stddev();

            // For each point in interpolated list.
            for (const arma::vec2& point : horizonPoints) {
                int greenTop = 0;
                int greenCount = 0;

                // If bottom of image, assume object.
                if (point[1] == (height - 1)) {
                    if ((point[1] - greenHorizon.at(point[0])[1]) >= MIN_DISTANCE_FROM_HORIZON) {
                        obstaclePoints.push_back(point);
                    }
                }

                else {
                    // Scan from point to bottom of image.
                    for (int y = point[1]; y < height; y++) {
                        /*
TODO: Sort out dependance on LookUpTable and Image.
                        if (getColourFromIndex(LUT.classifyPixel(img(p.x,y))) == green) {
                            if (greenCount == 1) {
                                greenTop = y;
                            }

                            greenCount++;
                            
                            // If VER_THRESHOLD green pixels found outside of acceptable range, add point.
                            if (greenCount == VER_THRESHOLD) {
                                if (greenTop > (mean_y + OBJECT_THRESHOLD_MULT * std_dev_y + 1)) {
                                    // Only add point if it is outside of minimum distance.
                                    if ((y - greenHorizon.at(point[0])[1]) >= MIN_DISTANCE_FROM_HORIZON) {
                                        arma::vec2 pushPoint;
                                        pushPoint << point[0] << y;
                                        obstaclePoints.push_back(pushPoint);
                                    }
                                }

                                break;
                            }
                        }

                        else {
                            greenCount = 0; // not green - reset
                        }
                        */

                        // If bottom reached without green, add bottom point.
                        if (y == (height - 1)) {
                            if ((y - greenHorizon.at(point[0])[1]) >= MIN_DISTANCE_FROM_HORIZON) {
                                arma::vec2 pushPoint;
                                pushPoint << point[0] << y;
                                obstaclePoints.push_back(pushPoint);
                            }
                        }
                    }
                }
            }

            // Find obstacles from these points.
            int start = 0;
            int count = 0, bottom = 0;
            bool scanning = false;

            for (unsigned int i = 0; i < obstaclePoints.size(); i++) {
                if (!scanning) {
                    start = i;
                    scanning = true;
                    count = 0;
                    bottom = 0;
                }

                else {
                    if (((obstaclePoints.at(i)[0] - obstaclePoints.at(i - 1)[0]) == static_cast<int>(VERTICAL_SCANLINE_SPACING)) && (i < (obstaclePoints.size() - 1))) {
                        // Count while there are consecutive points.
                        count++;

                        if (obstaclePoints.at(i)[1] > bottom)
                            bottom = obstaclePoints.at(i)[1];
                    }

                    else {
                        // Non-consecutive found.
                        if (count > MIN_CONSECUTIVE_POINTS) {
                            // If there are enough then make an obstacle.
                            int l = obstaclePoints.at(start)[0] - VERTICAL_SCANLINE_SPACING;
                            int r = obstaclePoints.at(i - 1)[0] + VERTICAL_SCANLINE_SPACING;

                            int centre = (l + r) * 0.5;
                            int width = r - l;
                            int NO_HEIGHT = -1; // DOES NOTHING

                            // Generate new obstacle.
                            arma::vec2 point;
                            point << centre << bottom;
                            obstacles.push_back(Obstacle(point, width, NO_HEIGHT));
                        }

                        scanning = false;
                    }
                }
            }

            // NOW ATTEMPT TO FIND ROBOTS
            std::vector<ColourSegment> cyanSegments = vbb->getAllTransitions(TEAM_CYAN_COLOUR);
            std::vector<ColourSegment> magentaSegments = vbb->getAllTransitions(TEAM_MAGENTA_COLOUR);

            for (Obstacle& obstacle : obstacles) {
                int cyanCount = 0;
                int magentaCount = 0;

                double bottom = obstacle.getLocationPixels()[1];
                double left = obstacle.getLocationPixels()[1] - (obstacle.getScreenSize()[0] * 0.5);
                double right = obstacle.getLocationPixels()[1] + (obstacle.getScreenSize()[0] * 0.5);

                for (const ColourSegment& cyanSegment : cyanSegments) {
                    if ((cyanSegment.m_end[0] >= left) && (cyanSegment.m_end[0] <= right) && (cyanSegment.m_end[1] <= bottom) &&
                       (cyanSegment.m_start[0] >= left) && (cyanSegment.m_start[0] <= right) && (cyanSegment.m_start[1] <= bottom)) {
                        cyanCount++;
                    }
                }

                for (const ColourSegment& magentaSegment : magentaSegments) {
                    if ((magentaSegment.m_end[0] >= left) && (magentaSegment.m_end[0] <= right) && (magentaSegment.m_end[1] <= bottom) &&
                       (magentaSegment.m_start[0] >= left) && (magentaSegment.m_start[0] <= right) && (magentaSegment.m_start[1] <= bottom)) {
                        magentaCount++;
                    }
                }

                if ((cyanCount >= MIN_COLOUR_THRESHOLD) && (magentaCount <= MAX_OTHER_COLOUR_THRESHOLD)) {
                    // Cyan robot.
                    obstCcle.m_colour = TEAM_CYAN_COLOUR;
                }

                else if ((magentaCount >= MIN_COLOUR_THRESHOLD) && (cyanCount <= MAX_OTHER_COLOUR_THRESHOLD)) {
                    // Magenta robot.
                    obstacle.m_colour = TEAM_MAGENTA_COLOUR;
                }

                else {
                    // Unknown obstacle.
                    obstacle.m_colour = UNKNOWN_COLOUR;
                }
            }

            return obstacles;
        }

        void ObstacleDetector::appendEdgesFromSegments(const std::vector<ColourSegment>& segments, std::vector<arma::vec2>& pointList) {
            for (const auto& segment : segments) {
                pointList.push_back(segment.m_start);
                pointList.push_back(segment.m_end);
            }
        }
    }
}
