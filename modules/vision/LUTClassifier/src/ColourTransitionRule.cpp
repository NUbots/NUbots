/*
 * This file is part of ColourTransitionRule.
 *
 * ColourTransitionRule is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ColourTransitionRule is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ColourTransitionRule.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#include "ColourTransitionRule.h"

namespace modules {
	namespace vision {

		using messages::vision::Colour;
		using messages::vision::ColourSegment;
		using messages::vision::COLOUR_CLASS;
		using messages::vision::ClassifiedImage;
		
	    ColourTransitionRule::ColourTransitionRule() {
	    	// Empty constructor.
	    }	    		

	    ColourSegment ColourTransitionRule::nomatch = {Colour::invalid, 0,arma::zeros<arma::vec>(2), arma::zeros<arma::vec>(2), arma::zeros<arma::vec>(2)};

	    void ColourTransitionRule::loadRuleFromConfigInfo(std::string colours_before,
															std::string colour_middle,
															std::string colours_after,
															unsigned int before_min,
															unsigned int before_max,
															unsigned int min,
															unsigned int max,
															unsigned int after_min,
															unsigned int after_max)	{
	    	//Clear current settings
	    	m_before.clear();
	    	m_middle.clear();
	    	m_after.clear();
	    	m_colour_class = COLOUR_CLASS::UNKNOWN_COLOUR;

	    	//Assign limits
	    	m_before_min = before_min;
			m_before_max = before_max;
			m_min = min;
			m_max = max;
			m_after_min = after_min;
			m_after_max = after_max;

			//Load rule colours
			//Initialise stream variables
			std::stringstream sstream;
			std::string current_colour_name;

			//Load before colours
			sstream << colours_before;			
			sstream >> current_colour_name;

			//While stream is not empty, check if the next word names a colour and load if it does. Get next word.
			while (!current_colour_name.empty()) {
				Colour colour = ClassifiedImage::getColourFromName(current_colour_name);

				if(colour != Colour::invalid) {
					m_before.push_back(colour);
				}

				sstream >> current_colour_name;
			}

			//We only support one middle colour which gives our colour class (ie one object)	
			Colour colour = ClassifiedImage::getColourFromName(colour_middle);
			m_colour_class = ClassifiedImage::getClassOfColour(colour);

			if(colour != Colour::invalid) {
				m_middle.push_back(colour);
			}

			else {
				//TODO: Log through NUClear
				std::cout<< "===================ERROR==================="<<std::endl;
				std::cout<< "Middle transition colour " << colour_middle << " not recognised."<<std::endl;
				std::cout<< "Note that only one middle colour can be specified for a transition rule."<<std::endl;
				std::cout<< "===================ERROR==================="<<std::endl;
			}
			
			//Load after colours
			sstream << colours_after;			
			sstream >> current_colour_name;

			//While stream is not empty, check if the next word names a colour and load if it does. Get next word.
			while (!current_colour_name.empty()) {
				Colour colour = ClassifiedImage::getColourFromName(current_colour_name);

				if (colour != Colour::invalid) {
					m_after.push_back(colour);
				}

				sstream >> current_colour_name;
			}			
		}
		
	    /*!
	      Checks if the given segment pair matches this rule (forward and reverse).
	      @param before the preceding segment.
	      @param middle the middle segment.
	      @param after the following segment.
	      @return Whether it is a match in either direction.
	      */
	    bool ColourTransitionRule::match(const ColourSegment &before, const ColourSegment& middle, const ColourSegment &after) const {
		    return (oneWayMatch(before, middle, after) || oneWayMatch(after, middle, before)); //test both directions
		}
		
	    //! Returns the ID of the field object that this rule is for.
		COLOUR_CLASS ColourTransitionRule::getColourClass() const {
		    return m_colour_class;
		} 

		bool ColourTransitionRule::oneWayMatch(const ColourSegment &before, const ColourSegment &middle, const ColourSegment &after) const {
		    //check lengths first to save iterating over colour vectors pointlessly as this method is majority false
		    if (!((m_min <= middle.m_lengthPixels) && (m_max >= middle.m_lengthPixels) &&
		         (m_before_min <= before.m_lengthPixels) && (m_before_max >= before.m_lengthPixels) &&
		         (m_after_min <= after.m_lengthPixels) && (m_after_max >= after.m_lengthPixels))) {
		        //did not match size requirements
		        return false;
		    }

		    bool valid;

		    if (!m_middle.empty()) {
		        if (middle.m_colour == Colour::invalid) {
		            return false;   //there is a before set, but no before colour
				}
				
		        valid = false;
				
				for (auto it : m_middle) {
		            if(it == middle.m_colour ){
		                valid = true;   //a match has been found
					}
		        }
				
		        if (!valid) {
		            return false;   //did not match before set
				}
		    }

		    if (!m_before.empty()) {
		        if (before.m_colour == Colour::invalid) {
		            return false;   //there is a before set, but no before colour
				}
				
		        valid = false;
				
				for (auto it : m_before) {
		            if(it == before.m_colour ){
		                valid = true;   //a match has been found
					}
		        }
				
		        if (!valid) {
		            return false;   //did not match before set
				}
		    }

		    if (!m_after.empty()) {
		        if (after.m_colour == Colour::invalid) {
		            return false;   //there is an after set, but no after colour
				}
				
		        valid = false;
				
				for (auto it : m_after) {
		            if (it == after.m_colour ){
		                valid = true;   //a match has been found
					}
		        }
				
		        if (!valid) {
		            return false;   //did not match after set
				}
		    }

		    return true;    //passed all checks
		}	   
	}
}