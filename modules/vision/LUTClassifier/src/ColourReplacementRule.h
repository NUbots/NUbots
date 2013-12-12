/*
 * This file is part of ColourReplacementRule.
 *
 * ColourReplacementRule is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ColourReplacementRule is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ColourReplacementRule.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
 
#ifndef MODULES_VISION_COLOURREPLACEMENTRULE_H
#define MODULES_VISION_COLOURREPLACEMENTRULE_H

#include <nuclear> 
#include <armadillo>
#include <string>
#include <vector>
#include <istream>
#include <ostream>
#include <iostream>

#include "messages/vision/ClassifiedImage.h"
#include "utility/strutil/strutil.h"

#include "LookUpTable.h"

namespace modules {
	namespace vision {
	
		class ColourReplacementRule {
		public:
			static messages::vision::ColourSegment nomatch;   //! @variable a static segment used to represent one that cannot be matched to any rule.
    
			enum ReplacementMethod {
		        BEFORE,
		        AFTER,
		        SPLIT,
		        INVALID
		    };

		    ColourReplacementRule();
		    
			void loadRuleFromConfigInfo(
	    		std::string colours_before,
		    	std::string colour_middle,
		    	std::string colours_after,
				unsigned int before_min,
				unsigned int before_max,
				unsigned int min,
				unsigned int max,
				unsigned int after_min,
				unsigned int after_max,
				std::string replacement_method);
			/*!
				Checks if the given segment triplet matches this rule.
				@param before the first segment.
				@param middle the second segment.
				@param after the last segment.
				@param dir The scan direction (vertical or horizontal).
				@return Whether it is a match.
				*/
    		bool match(const messages::vision::ColourSegment& before, const messages::vision::ColourSegment& middle, const messages::vision::ColourSegment& after) const;
    
			/*!
			  Gets the method matching the given string.
			  @param name String name of the method.
			  @return The method desired.
			  */
			ReplacementMethod getMethodFromName(const std::string& name) const;
			
			/*!
			  Gets the name of the given method.
			  @param method The method name desired.
			  @return String name of the method.
			  */
    		std::string getMethodName(const ReplacementMethod& method) const;
			
			/*!
			  Returns the replacement method (before, after or split) for this rule.
			   - before - the middle segment is given the colour of the first.
			   - after - the middle segment is given the colour of the last.
			   - split - the middle segment is split into two, each given the colour of the adjacent segment.
			  @return An enum for the method.
			  */
    		ReplacementMethod getMethod() const;

    	private:
   			std::string m_name;  //! @variable the name of the rule.
    
			ReplacementMethod m_method;  //! @variable The replacement method for this rule.

       		std::vector<messages::vision::Colour> m_before;				//! @variable The colour that the first segment must be.
			std::vector<messages::vision::Colour> m_middle;				//! @variable The colour that the middle segment must be.
			std::vector<messages::vision::Colour> m_after;				//! @variable The colour that the last segment must be.

			unsigned int m_middle_min;									//! @variable the minimum length of the middle segment for a match.
		    unsigned int m_middle_max;									//! @variable the maximum length of the middle segment for a match.
		    unsigned int m_before_min;									//! @variable the minimum length of the first segment for a match.
		    unsigned int m_before_max;									//! @variable the maximum length of the first segment for a match.
		    unsigned int m_after_min;									//! @variable the minimum length of the last segment for a match.
		    unsigned int m_after_max;									//! @variable the maximum length of the last segment for a match. 
						 
			bool oneWayMatch(const messages::vision::ColourSegment& before, const messages::vision::ColourSegment& middle, const messages::vision::ColourSegment& after) const;
		};
		
	} // vision
} // modules

#endif // MODULES_VISION_COLOURREPLACEMENTRULE_H