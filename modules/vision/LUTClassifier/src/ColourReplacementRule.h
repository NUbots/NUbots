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
 
#ifndef COLOURREPLACEMENTRULE_H
#define COLOURREPLACEMENTRULE_H

#include <armadillo>
#include "ColourSegment.h"
#include "messages/vision/ClassifiedImage.h"


modules {
	vision {
		class ColourReplacementRules {
		public:
			enum ReplacementMethod {
		        BEFORE,
		        AFTER,
		        SPLIT,
		        INVALID
		    };

		    ColourReplacementRules(arma::vec2 vec, string colour);

			/*!
				Checks if the given segment triplet matches this rule.
				@param before the first segment.
				@param middle the second segment.
				@param after the last segment.
				@param dir The scan direction (vertical or horizontal).
				@return Whether it is a match.
				*/
    		bool match(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after) const;
			
			
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

       		std::vector<messages::vision::ClassifiedImage::Colour>  m_before,   //! @variable The colour that the first segment must be.
			                     m_middle,   //! @variable The colour that the middle segment must be.
			                     m_after;    //! @variable The colour that the last segment must be.

			unsigned int m_middle_min,   //! @variable the minimum length of the middle segment for a match.
		                 m_middle_max,   //! @variable the maximum length of the middle segment for a match.
		                 m_before_min,   //! @variable the minimum length of the first segment for a match.
		                 m_before_max,   //! @variable the maximum length of the first segment for a match.
		                 m_after_min,    //! @variable the minimum length of the last segment for a match.
		                 m_after_max;    //! @variable the maximum length of the last segment for a match. 
			bool oneWayMatch(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after) const;		                 
		};
	}
}

#endif