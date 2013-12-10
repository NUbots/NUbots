/*
 * This file is part of LUTClassifier.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ClassifiedImage.h"


namespace messages {
    namespace input {
        ClassifiedImage::ClassifiedImage(){}      
	    std::string ClassifiedImage::getColourName(const Colour& colour) {
	            switch(colour) {
	                case unclassified: {
	                    return "unclassified";
	                }

	                case white: {
	                    return "white";
	                }

	                case green: {
	                    return "green";
	                }

	                case shadow_object: {
	                    return "shadow object";
	                }

	                case pink: {
	                    return "pink";
	                }

	                case pink_orange: {
	                    return "pink - orange";
	                }

	                case orange: {
	                    return "orange";
	                }

	                case yellow_orange: {
	                    return "yellow - orange";
	                }

	                case yellow: {
	                    return "yellow";
	                }

	                case blue: {
	                    return "blue";
	                }

	                case shadow_blue: {
	                    return "shadow blue";
	                }

	                default: {
	                    return "unknown colour!";
	                }
	            }
	        }
    }  // input
}  // messages
