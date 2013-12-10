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
#include "ColourTransitionRule.h"

modules {
	vision {
		using messages::vision::ClassifiedImage::COLOUR_CLASS;
		using messages::vision::ClassifiedImage::Colour;

	    ColourTransitionRule::ColourTransitionRule(){}
	    /*!
	      Checks if the given segment pair matches this rule (forward and reverse).
	      @param before the preceeding segment.
	      @param middle the middle segment.
	      @param after the following segment.
	      @return Whether it is a match in either direction.
	      */
	    bool ColourTransitionRule::match(const ColourSegment &before, const ColourSegment& middle, const ColourSegment &after) const{
		    return oneWayMatch(before, middle, after) || oneWayMatch(after, middle, before); //test both directions
		}
	    //! Returns the ID of the field object that this rule is for.
	    COLOUR_CLASS ColourTransitionRule::getColourClass() const{
		    return m_colour_class;
		} 

		bool ColourTransitionRule::oneWayMatch(const ColourSegment &before, const ColourSegment &middle, const ColourSegment &after) const
		{
		    //check lengths first to save iterating over colour vectors pointlessly as this method is majority false
		    if(!(m_min <= middle.getLength() && m_max >= middle.getLength() &&
		         m_before_min <= before.getLength() && m_before_max >= before.getLength() &&
		         m_after_min <= after.getLength() && m_after_max >= after.getLength())) {
		        //did not match size requirements
		        return false;
		    }

		    bool valid;
		    std::vector<Colour>::const_iterator it;

		    if(!m_middle.empty()) {
		        if(middle.getColour() == invalid)
		            return false;   //there is a before set, but no before colour
		        valid = false;
		        for(it = m_middle.begin(); it != m_middle.end(); it++) {
		            if(*it == middle.getColour())
		                valid = true;   //a match has been found
		        }
		        if(!valid)
		            return false;   //did not match before set
		    }

		    if(!m_before.empty()) {
		        if(before.getColour() == invalid)
		            return false;   //there is a before set, but no before colour
		        valid = false;
		        for(it = m_before.begin(); it != m_before.end(); it++) {
		            if(*it == before.getColour())
		                valid = true;   //a match has been found
		        }
		        if(!valid)
		            return false;   //did not match before set
		    }

		    if(!m_after.empty()) {
		        if(after.getColour() == invalid)
		            return false;   //there is an after set, but no after colour
		        valid = false;
		        for(it = m_after.begin(); it != m_after.end(); it++) {
		            if(*it == after.getColour())
		                valid = true;   //a match has been found
		        }
		        if(!valid)
		            return false;   //did not match after set
		    }

		    return true;    //passed all checks
		}	   
	}
}