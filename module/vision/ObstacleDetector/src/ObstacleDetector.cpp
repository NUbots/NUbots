/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ObstacleDetector.h"

#include <armadillo>

#include "message/vision/ClassifiedImage.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/vision/ClassifiedImage.h"

namespace module {
namespace vision {

    using message::vision::ClassifiedImage;
    using SegmentClass = ClassifiedImage::SegmentClass;

    ObstacleDetector::ObstacleDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<ClassifiedImage>, Single>().then("Obstacle Detector", [this] (const ClassifiedImage& image) {

            std::vector<arma::ivec2> points;

            // Get all the segments that are relevant to finding an obstacle
            for(int i = 0; i < 6; ++i) 
            {
                std::vector<ClassifiedImage::Segment> segments;
                const auto& sourceSegments = ((i % 2) == 0) ? image.horizontalSegments : image.verticalSegments;

                std::copy_if(sourceSegments.begin(), sourceSegments.end(), segments.end(), [&] (const ClassifiedImage::Segment& segment) -> bool
                {
                    if (((i == 0) || (i == 1)) && (segment.segmentClass.value == SegmentClass::UNKNOWN_CLASS))
                    {
                        return(true);
                    }
                    
                    if (((i == 2) || (i == 3)) && (segment.segmentClass.value == SegmentClass::CYAN_TEAM))
                    {
                        return(true);
                    }
                    
                    if (((i == 4) || (i == 5)) && (segment.segmentClass.value == SegmentClass::MAGENTA_TEAM))
                    {
                        return(true);
                    }

                    return(false);
                });

                for (const auto& segment : segments)
                {
                    const auto& start = segment.start;
                    const auto& end   = segment.end;

                    // Check if we have a subsequent segment
                    if(segment.previous && utility::vision::visualHorizonAtPoint(image, start[0]) < start[1]) {
                        points.push_back(convert<int, 2>(start));
                    }

                    // Check if we have a subsequent segment
                    if(segment.next && utility::vision::visualHorizonAtPoint(image, end[0]) < end[1]) {
                        points.push_back(convert<int, 2>(end));
                    }
                }
            }

            // Sort our points
            std::sort(points.begin(), points.end(), [] (const arma::ivec2& a, const arma::ivec2& b) {
                return a[0] < b[0];
            });

            // Now build a locally convex hull or something

            // Or possibly do a threshold check (for every start there is an end, if all segments end we have reached the end of the obstacle)

            //JAKE's Vision Kinematics for distance to obstacle:
            // arma::vec2 p1 = imageToScreen(obstacleBaseCentreImage, { double(image.dimensions[0]), double(image.dimensions[1]) });

            // arma::vec3 obstacleBaseGroundProj = projectCamToGroundPlane(p1, sensors.orientationCamToGround);
            // //Testing (not done yet - TODO: TEST AND REMOVE THIS NOTE)
            // std::cout << "orientationCamToGround\n" << sensors.orientationCamToGround << std::endl;
            // std::cout << "D2P obstacle: " << obstacleBaseGroundProj.t() << std::endl;
            // emit(graph("D2P Obstacle", obstacleBaseGroundProj[0], obstacleBaseGroundProj[1]));


        });

    }

}
}

