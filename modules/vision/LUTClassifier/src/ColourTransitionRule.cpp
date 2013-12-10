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
		using messages::vision::ClassifiedImage;

	    ColourTransitionRule::ColourTransitionRule() {
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
	    ClassifiedImage::COLOUR_CLASS ColourTransitionRule::getColourClass() const {
		    return m_colour_class;
		} 

		bool ColourTransitionRule::oneWayMatch(const ColourSegment &before, const ColourSegment &middle, const ColourSegment &after) const {
		    //check lengths first to save iterating over colour vectors pointlessly as this method is majority false
		    if (!((m_min <= middle.getLength()) && (m_max >= middle.getLength()) &&
		         (m_before_min <= before.getLength()) && (m_before_max >= before.getLength()) &&
		         (m_after_min <= after.getLength()) && (m_after_max >= after.getLength()))) {
		        //did not match size requirements
		        return false;
		    }

		    bool valid;

		    if (!m_middle.empty()) {
		        if (middle.getColour() == ClassifiedImage::invalid) {
		            return false;   //there is a before set, but no before colour
				}
				
		        valid = false;
				
				for (auto it : m_middle) {
		            if(it == middle.getColour()) {
		                valid = true;   //a match has been found
					}
		        }
				
		        if (!valid) {
		            return false;   //did not match before set
				}
		    }

		    if (!m_before.empty()) {
		        if (before.getColour() == ClassifiedImage::invalid) {
		            return false;   //there is a before set, but no before colour
				}
				
		        valid = false;
				
				for (auto it : m_before) {
		            if(it == before.getColour()) {
		                valid = true;   //a match has been found
					}
		        }
				
		        if (!valid) {
		            return false;   //did not match before set
				}
		    }

		    if (!m_after.empty()) {
		        if (after.getColour() == ClassifiedImage::invalid) {
		            return false;   //there is an after set, but no after colour
				}
				
		        valid = false;
				
				for (auto it : m_after) {
		            if (it == after.getColour()) {
		                valid = true;   //a match has been found
					}
		        }
				
		        if (!valid) {
		            return false;   //did not match after set
				}
		    }

		    return true;    //passed all checks
		}	   

		/*! @brief Stream insertion operator for a single ColourTransitionRule
		 */
		std::ostream& operator<< (std::ostream& output, const ColourTransitionRule& c) {
			output << ClassifiedImage::getColourClassName(c.m_colour_class) << ":\n";

			//before
			output << "before: (" << c.m_before_min << ", " << c.m_before_max << ") [";
			
			for (auto it : c.m_before) {
				output << LookUpTable::getColourName(it) << ", ";
			}
			
			output << "]\t// (min, max) [colourlist]\n";

			//this
			output << "middle: (" << c.m_min << ", " << c.m_max << ") [";
			
			for (auto it : c.m_middle) {
				output << LookUpTable::getColourName(it) << ", ";
			}
			
			output << "]\t// (min, max) [colourlist]\n";

			//after
			output << "after: (" << c.m_after_min << ", " << c.m_after_max << ") [";
			
			for (auto it : c.m_after) {
				output << LookUpTable::getColourName(it) << ", ";
			}
			
			output << "]\t// (min, max) [colourlist]" << std::endl;

			return output;
		}

		/*! @brief Stream insertion operator for a vector of ColourTransitionRule.
		 *  @relates ColourRule
		 */
		std::ostream& operator<< (std::ostream& output, const std::vector<ColourTransitionRule>& v) {
			for (auto it : v) {		
				output << it;
			}
			
			return output;
		}

		/*! @brief Stream extraction operator for a ColourTransitionRule.
		 *  @relates ColourRule
		 */
		std::istream& operator>> (std::istream& input, ColourTransitionRule& c) {
			std::stringstream colour_stream;
			std::string next, colour_str;
			std::string id_str;

			// read in the rule name
			std::getline(input, id_str, ':');
			id_str = id_str.erase(0, id_str.find_first_not_of(' '));		// remove spaces from the start of the string.
			id_str = id_str.erase(id_str.find_last_not_of(' '));			// remove spaces from the end of the string.
			c.m_colour_class = ClassifiedImage::getColourClassFromName(id_str);

			//BEFORE
			//reset colour list
			c.m_before.clear();
			
			// read in the before: (min, max)
			input.ignore(30, '(');
			input >> c.m_before_min;
			input.ignore(10, ',');
			input >> c.m_before_max;
			input.ignore(10, ')');

			input.ignore(10, '[');

			//get colour list
			std::getline(input, colour_str, ']');
			colour_str.erase(std::remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
			
			if (!colour_str.empty()) {
				colour_stream.str(colour_str);
				
				while(colour_stream.good()) {
					std::getline(colour_stream, next, ',');
					c.m_before.push_back(LookUpTable::getColourFromName(next));
				}
			}

			//MIDDLE
			//reset colour list
			c.m_middle.clear();
			
			// read in the before: (min, max)
			input.ignore(30, '(');
			input >> c.m_min;
			input.ignore(10, ',');
			input >> c.m_max;
			input.ignore(10, ')');

			input.ignore(10, '[');

			//get colour list
			std::getline(input, colour_str, ']');
			colour_str.erase(std::remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
			
			if (!colour_str.empty()) {
				colour_stream.str(colour_str);
				
				while(colour_stream.good()) {
					std::getline(colour_stream, next, ',');
					c.m_middle.push_back(LookUpTable::getColourFromName(next));
				}
			}

			//AFTER
			//reset colour list
			c.m_after.clear();
			
			// read in the before: (min, max)
			input.ignore(30, '(');
			input >> c.m_after_min;
			input.ignore(10, ',');
			input >> c.m_after_max;
			input.ignore(10, ')');

			input.ignore(10, '[');

			//get colour list
			std::getline(input, colour_str, ']');
			colour_str.erase(std::remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
			
			if(!colour_str.empty()) {
				colour_stream.clear();
				colour_stream.str(colour_str);
				
				while(colour_stream.good()) {
					std::getline(colour_stream, next, ',');
					c.m_after.push_back(LookUpTable::getColourFromName(next));
				}
			}

			// ignore the rest of the line
			input.ignore(128, '\n');
			input.peek();               //trigger eofbit being set in the case of this being the last rule

			return input;
		}

		/*! @brief Stream extraction operator for a vector of ColourTransitionRule.
		 *  @relates ColourTransitionRule
		 */
		std::istream& operator>> (std::istream& input, std::vector<ColourTransitionRule>& v) {
			ColourTransitionRule temp;
			v.clear();
			
			while(input.good()) {
				input >> temp;
				
				if(temp.getColourClass() != ClassifiedImage::UNKNOWN_COLOUR) {
					v.push_back(temp);
				}
				
				else {
					std::cout << "ColourTransitionRule istream operator: UNKOWN_COLOUR match ignored." << std::endl;
				}
			}

			return input;
		}
	}
}