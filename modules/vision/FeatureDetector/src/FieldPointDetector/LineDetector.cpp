/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "LineDetector.h"

namespace modules {
    namespace vision {

        LineDetector::LineDetector() {
            //Empty Constructor.
        }

        LineDetector::~LineDetector() {
            // Empty destructor.
        }

        std::vector<FieldLine> LineDetector::run(const std::vector<NUPoint>& points) {
            std::vector<FieldLine> temp;
            return temp;
        }

        /// @note this merges based on the first line, so ordering them is important
        std::vector<std::pair<LSFittedLine, LSFittedLine> > LineDetector::mergeColinear(std::vector<std::pair<LSFittedLine, LSFittedLine> > lines,
                                                                              double angleThreshold, double distanceThreshold) const {
            // O(l^2)  -  l = number of lines.
            // Compares all lines and merges based on the angle between and the average distance between.

            std::vector<std::pair<LSFittedLine, LSFittedLine>> finals;  // This std::vector contains lines that have been merged or did not need to be.
            std::pair<LSFittedLine, LSFittedLine> current;              // Line currently being compared with the rest.

            while (!lines.empty()) {
                // Get next line.
                current = lines.back();
                lines.pop_back();

                std::vector<std::pair<LSFittedLine, LSFittedLine>>::iterator it = lines.begin();
                
                // Go through all lines and find any that should be merged - merge them.
                for (auto it = lines.begin(); it != lines.end(); /* Iteratation done by if statement. */ ) {
                    if ((current.first.getAngleBetween(it->first) <= angleThreshold) &&
                        (current.first.averageDistanceBetween(it->first) <= distanceThreshold)) {
                        current.first.joinLine(it->first);          // Join the other line to current.
                        current.second.joinLine(it->second);        // Join the other paired lines.
                        it = lines.erase(it);                       // Remove the other line.
                    }
                    
                    else {
                        it++;
                    }
                }
                
                // Now current should have been merged with any valid lines
                // push current to finals
                finals.push_back(current);
            }

            return finals;
        }

    }
}

