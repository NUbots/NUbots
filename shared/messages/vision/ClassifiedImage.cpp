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
    namespace vision {
	
        ClassifiedImage::ClassifiedImage() {
		}      

		std::string getColourClassName(const ClassifiedImage::COLOUR_CLASS& id) {
			switch (id) {
				case ClassifiedImage::BALL_COLOUR: {
					return "BALL_COLOUR";
				}
					
				case ClassifiedImage::GOAL_COLOUR: {
					return "GOAL_COLOUR";
				}

/*				
				case ClassifiedImage::GOAL_Y_COLOUR: {
					return "GOAL_Y_COLOUR";
				}
					
				case ClassifiedImage::GOAL_B_COLOUR: {
					return "GOAL_B_COLOUR";
				}
*/

				case ClassifiedImage::LINE_COLOUR: {
					return "LINE_COLOUR";
				}
					
				default: {
					return "UNKNOWN_COLOUR";
				}
			}
		}
    
		ClassifiedImage::COLOUR_CLASS getColourClassFromName(const std::string& name) {
			if (name.compare("BALL_COLOUR") == 0) {
				return ClassifiedImage::BALL_COLOUR;
			}
			
			else if (name.compare("GOAL_COLOUR") == 0) {
				return ClassifiedImage::GOAL_COLOUR;
			}
			
/*			
			else if (name.compare("GOAL_Y_COLOUR") == 0) {
				return ClassifiedImage::GOAL_Y_COLOUR;
			}
			
			else if (name.compare("GOAL_B_COLOUR") == 0) {
				return ClassifiedImage::GOAL_B_COLOUR;
			}
*/
		
			else if (name.compare("LINE_COLOUR") == 0) {
				return ClassifiedImage::LINE_COLOUR;
			}
				
			else {
				return ClassifiedImage::UNKNOWN_COLOUR;
			}
		}

    }  // messages
}  // messages
