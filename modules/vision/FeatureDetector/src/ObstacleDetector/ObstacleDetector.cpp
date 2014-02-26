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
        using utility::vision::LookUpTable;
        using messages::input::Image;
        using messages::vision::ClassifiedImage;

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

            VER_THRESHOLD = VER_THRESHOLD_;
            OBJECT_THRESHOLD_MULT = OBJECT_THRESHOLD_MULT_;
        }

        std::unique_ptr< std::vector<messages::vision::Obstacle> > ObstacleDetector::run(const std::vector<arma::vec2>& greenHorizon,
                                                    const LookUpTable& LUT,
                                                    const Image& img,
                                                    const std::vector<ColourSegment>& cyanSegments,
                                                    const std::vector<ColourSegment>& magentaSegments,
                                                    const VisionKinematics& visionKinematics) {
            //std::cout << "ObstacleDetector::run - " <<  debugCounter++ << std::endl;          

            std::vector<arma::vec2> obstaclePoints;
            obstaclePoints.reserve(greenHorizon.size());
            std::vector<Obstacle> obstacles;

            int imgHeight = img.height();
            double meanY, stdDevY;
            //std::cout << "ObstacleDetector::run - " <<  debugCounter++ << std::endl;          

            //Get subset of green horizon points
            std::vector<arma::vec2> horizonPoints;
            for(size_t i = 0; i < greenHorizon.size() ; i+= VERTICAL_SCANLINE_SPACING){
                horizonPoints.push_back(greenHorizon[i]);
            }

            // Calculate mean and stddev of vertical positions.
            arma::running_stat<double> accY;

            for(const arma::vec2& point : horizonPoints) {
                accY(point[1]);
            }
            //std::cout << "ObstacleDetector::run - " <<  debugCounter++ << std::endl;
            //Segfault after here          

            meanY = accY.mean();
            stdDevY = accY.stddev();

            // For each point in interpolated list.
            for (const arma::vec2& point : horizonPoints) {
                //std::cout << "ObstacleDetector::run - scanning from hor point "<< point << std::endl;  
                //std::cout << "..." << std::endl;  

                int greenTop = 0;
                int greenCount = 0;

                // If bottom of image, assume object.
                if (point[1] == (imgHeight - 1)) {
                    if (point[1] - greenHorizon.at(point[0])[1] >= MIN_DISTANCE_FROM_HORIZON) {
                        //std::cout << "ObstacleDetector::run - point at bottom of image "<< std::endl;  
                        obstaclePoints.insert(obstaclePoints.end(),point);
                    }
                }
                else {
                    // Scan from point to bottom of image.
                    for (int y = point[1]; y < imgHeight; y++) {
                        //std::cout << "ObstacleDetector::run - scanning: y = "<< y << " /" << imgHeight << std::endl;  

                        if (ClassifiedImage::getClassOfColour(LUT.classifyPixel(img(point[0],y))) == COLOUR_CLASS::FIELD_COLOUR) {
                            //std::cout << "ObstacleDetector::run - pixel field colour "<< std::endl;  

                            if (greenCount == 1) {
                                greenTop = y;
                            }

                            greenCount++;
                            
                            // If VER_THRESHOLD green pixels found outside of acceptable range, add point.
                            if (greenCount >= VER_THRESHOLD) {
                                //std::cout << "ObstacleDetector::run - greenCount >= VER_THRESHOLD ("<< greenCount << " >= "<< VER_THRESHOLD<< ")" << std::endl;  

                                if (greenTop > (meanY + OBJECT_THRESHOLD_MULT * stdDevY + 1)) {
                                    //std::cout << "ObstacleDetector::run - greenTop > (meanY + OBJECT_THRESHOLD_MULT * stdDevY + 1) ("<< greenTop << " > "<< (meanY + OBJECT_THRESHOLD_MULT * stdDevY + 1)<< ")" << std::endl;  
                                    //std::cout << "ObstacleDetector::run - greenHorizon.at(static_cast<int>(point[0]))[1]) = "<< greenHorizon.at(static_cast<int>(point[0]))[1] << std::endl;  

                                    // Only add point if it is outside of minimum distance.
                                    if ((y - greenHorizon.at(static_cast<int>(point[0]))[1]) >= MIN_DISTANCE_FROM_HORIZON) {
                                        //std::cout << "ObstacleDetector::run - greenHorizon.at(static_cast<int>(point[0]))[1]) >= MIN_DISTANCE_FROM_HORIZON  ("<< greenHorizon.at(static_cast<int>(point[0]))[1] << " > "<<MIN_DISTANCE_FROM_HORIZON<< ")" << std::endl;  
                                        double y_ = y;
                                        arma::vec2 pushPoint = { point[0] , y_};                                       
                                        //std::cout << "ObstacleDetector::run - push_back " << pushPoint << std::endl;
                                        obstaclePoints.insert(obstaclePoints.end(),pushPoint);
                                        //std::cout << "ObstacleDetector::run - push_back done "<< std::endl;

                                    }
                                }
                                //std::cout << "ObstacleDetector::run - breaking scan" << std::endl;

                                break;
                            }
                        }

                        else {
                            greenCount = 0; // not green - reset
                        }
                        

                        // If bottom reached without green, add bottom point.
                        if (y == (imgHeight - 1)) {
                            if ((y - greenHorizon.at(static_cast<int>(point[0]))[1]) >= MIN_DISTANCE_FROM_HORIZON) {
                                //std::cout << "ObstacleDetector::run - bottom of image reached without green "<< std::endl;  
                                double y_ = y;
                                arma::vec2 pushPoint = { point[0] , y_};
                                obstaclePoints.insert(obstaclePoints.end(),pushPoint);
                            }
                        }
                    }
                }
            }
            ////std::cout  << "Found obstacle points:"<< obstaclePoints.size() <<std::endl;            
            // Find obstacles from these points.
            int start = 0;
            int count = 0, bottom = 0;
            bool scanning = false;
            //std::cout << "ObstacleDetector::run - " <<  debugCounter++ << std::endl;          

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

                        if (obstaclePoints.at(i)[1] > bottom){
                            bottom = obstaclePoints.at(i)[1];
                        }
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
                            obstacles.push_back(Obstacle(visionKinematics, point, width, NO_HEIGHT));
                        }

                        scanning = false;
                    }
                }
            }
            //std::cout << "ObstacleDetector::run - " <<  debugCounter++ << std::endl;          

            // NOW ATTEMPT TO FIND ROBOTS        
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
                    obstacle.m_colour = COLOUR_CLASS::TEAM_CYAN_COLOUR;
                }

                else if ((magentaCount >= MIN_COLOUR_THRESHOLD) && (cyanCount <= MAX_OTHER_COLOUR_THRESHOLD)) {
                    // Magenta robot.
                    obstacle.m_colour = COLOUR_CLASS::TEAM_MAGENTA_COLOUR;
                }

                else {
                    // Unknown obstacle.
                    obstacle.m_colour = COLOUR_CLASS::UNKNOWN_COLOUR;
                }
            }
            

            return std::move(createObstacleMessage(obstacles));
        }

        void ObstacleDetector::appendEdgesFromSegments(const std::vector<ColourSegment>& segments, std::vector<arma::vec2>& pointList) {
            for (const auto& segment : segments) {
                pointList.push_back(segment.m_start);
                pointList.push_back(segment.m_end);
            }
        }

        std::unique_ptr< std::vector<messages::vision::Obstacle> > ObstacleDetector::createObstacleMessage(std::vector<Obstacle> obstacles){
            std::unique_ptr< std::vector<messages::vision::Obstacle> > obstacle_message = std::make_unique< std::vector<messages::vision::Obstacle> >();
            for(auto& obstacle : obstacles){
                obstacle_message->push_back(messages::vision::Obstacle());
                //NUClear::log<NUClear::DEBUG>("Found ",obstacle);
                if (obstacle.isValid()) {
                    obstacle_message->back().sphericalFromNeck = obstacle.m_location.neckRelativeRadial;
                    obstacle_message->back().sphericalError = obstacle.m_sphericalError;
                    obstacle_message->back().screenAngular = obstacle.m_location.screenAngular;
                    obstacle_message->back().screenCartesian = obstacle.m_location.screenCartesian;
                    obstacle_message->back().sizeOnScreen = obstacle.m_sizeOnScreen;
                    obstacle_message->back().timestamp = NUClear::clock::now();
                    
                    obstacle_message->back().arcWidth = obstacle.m_arcWidth;
                    obstacle_message->back().colour = obstacle.m_colour;
                }
            }
            return std::move(obstacle_message);
        }


    }
}
