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

#include "ColourReplacementRule.h"
 
namespace modules {
    namespace vision {

		using messages::vision::Colour;
		using messages::vision::ClassifiedImage;
		using messages::vision::ColourSegment;
		
		ColourSegment ColourReplacementRule::nomatch(arma::zeros<arma::vec>(2), arma::zeros<arma::vec>(2), invalid);

		ColourReplacementRule::ColourReplacementRule() {
			// Empty constructor.
		}


		void ColourReplacementRule::loadRuleFromConfigInfo(std::string colours_before,
															std::string colours_middle,
															std::string colours_after,
															unsigned int before_min,
															unsigned int before_max,
															unsigned int min,
															unsigned int max,
															unsigned int after_min,
															unsigned int after_max,
															std::string replacement_method)	{
	    	//Clear current settings
	    	m_before.clear();
	    	m_middle.clear();
	    	m_after.clear();

	    	//Assign limits
	    	m_before_min = before_min;
			m_before_max = before_max;
			m_middle_min = min;
			m_middle_max = max;
			m_after_min = after_min;
			m_after_max = after_max;

			//replacement method
			m_method = getMethodFromName(replacement_method);

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

				if (colour != Colour::invalid){
					m_before.push_back(colour);
				}

				sstream >> current_colour_name;
			}

			//Load middle colours
			sstream << colours_middle;			
			sstream >> current_colour_name;

			//While stream is not empty, check if the next word names a colour and load if it does. Get next word.
			while (!current_colour_name.empty()) {
				Colour colour = ClassifiedImage::getColourFromName(current_colour_name);

				if (colour != Colour::invalid){
					m_middle.push_back(colour);
				}

				sstream >> current_colour_name;
			}
			
			//Load after colours
			sstream << colours_after;			
			sstream >> current_colour_name;

			//While stream is not empty, check if the next word names a colour and load if it does. Get next word.
			while (!current_colour_name.empty()) {
				Colour colour = ClassifiedImage::getColourFromName(current_colour_name);

				if(colour != Colour::invalid){
					m_after.push_back(colour);
				}

				sstream >> current_colour_name;
			}			
		}

		std::string ColourReplacementRule::getMethodName(const ColourReplacementRule::ReplacementMethod& method) const {
			switch (method) {
				case BEFORE: {
					return "before";
				}
				
				case AFTER: {
					return "after";
				}
				
				case SPLIT: {
					return "green";
				}
				
				default: {
					return "unknown method";
				}
			};
		}

		/*!
		  Gets the method matching the given string.
		  @param name String name of the method.
		  @return The method desired.
		  */
		ColourReplacementRule::ReplacementMethod ColourReplacementRule::getMethodFromName(const std::string& name) const {
			if (name.compare("before") == 0) {
				return BEFORE;
			}
			
			else if (name.compare("after") == 0) {
				return AFTER;
			}
			
			else if (name.compare("split") == 0) {
				return SPLIT;
			}
			
			else {
				return INVALID;
			}
		}

		bool ColourReplacementRule::match(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after) const {
			//check lengths first to save iterating over colour vectors pointlessly as this method is majority false
			if (!((m_middle_min <= middle.m_lengthPixels) && (m_middle_max >= middle.m_lengthPixels) &&
				 (m_before_min <= before.m_lengthPixels) && (m_before_max >= before.m_lengthPixels) &&
				 (m_after_min <= after.m_lengthPixels) && (m_after_max >= after.m_lengthPixels))) {
				//did not match size requirements
				return false;
			}

			bool valid;
			
			if (!m_middle.empty()) {
				valid = false;
				
				for (auto it : m_middle) {
					if(it == middle.getColour())
						valid = true;   //a match has been found
				}
				
				if (!valid) {
					return false; //did not match middle set
				}
			}
			
			else {
				return false;	//if middle is empty the rule matches nothing
			}

			if (!m_before.empty()) {
				if (before.getColour() == Colour::invalid) {
					return false;   //there is a before set, but no before colour
				}
				
				valid = false;
				
				for (auto it : m_before) {
					if (it == before.getColour()) {
						valid = true;   //a match has been found
					}
				}
				
				if (!valid) {
					return false;   //did not match before set
				}
			}

			if (!m_after.empty()) {
				if (after.getColour() == Colour::invalid) {
					return false;   //there is an after set, but no after colour
				}
				
				valid = false;
				
				for (auto it : m_after) {
					if(it == after.getColour()) {
						valid = true;   //a match has been found
					}
				}
				
				if (!valid) {
					return false;   //did not match after set
				}
			}

			return true;    //passed all checks
		}

		ColourReplacementRule::ReplacementMethod ColourReplacementRule::getMethod() const {
			return m_method;
		}
	}
}