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
        using messages::vision::ClassifiedImage;
		
		ColourSegment nomatch(arma::zeros<arma::vec>(2), arma::zeros<arma::vec>(2), ClassifiedImage::invalid);

		ColourReplacementRule::ColourReplacementRule() {
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
			if (!((m_middle_min <= middle.getLength()) && (m_middle_max >= middle.getLength()) &&
				 (m_before_min <= before.getLength()) && (m_before_max >= before.getLength()) &&
				 (m_after_min <= after.getLength()) && (m_after_max >= after.getLength()))) {
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
				if (before.getColour() == ClassifiedImage::invalid) {
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
				if (after.getColour() == ClassifiedImage::invalid) {
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

		/*! @brief Stream insertion operator for a single ColourReplacementRule
		 */
		std::ostream& operator<< (std::ostream& output, const ColourReplacementRule& c) {
			output << c.m_name << ":" << std::endl;

			//before
			output << "\tbefore: (" << c.m_before_min << ", " << c.m_before_max << ") [";
			
			for (auto it : c.m_before) {
				output << ClassifiedImage::getColourName(it) << ", ";
			}
			
			output << "]\t\t// (min, max) [colourlist]" << std::endl;

			//middle
			output << "\tmiddle: (" << c.m_middle_min << ", " << c.m_middle_max << ") [";
			
			for (auto it : c.m_middle) {
				output << ClassifiedImage::getColourName(it) << ", ";
			}
			
			output << "]\t\t// (min, max) [colourlist]" << std::endl;

			//after
			output << "\tafter(" << c.m_after_min << ", " << c.m_after_max << ") [";
			
			for (auto it : c.m_after) {
				output << ClassifiedImage::getColourName(it) << ", ";
			}
			
			output << "]\t\t// (min, max) [colourlist]" << std::endl;

			//replacement method
			output << "\treplacement: " << c.getMethodName(c.m_method) << "\t\t// [colourlist]" << std::endl;

			return output;
		}

		/*! @brief Stream insertion operator for a vector of ColourReplacementRule.
		 *  @relates ColourReplacementRule
		 */
		std::ostream& operator<< (std::ostream& output, const std::vector<ColourReplacementRule>& v) {
			for (auto it : v) {
				output << it;
			}
			
			return output;
		}

		/*! @brief Stream extraction operator for a ColourReplacementRule.
		 *  @relates ColourReplacementRule
		 */
		std::istream& operator>> (std::istream& input, ColourReplacementRule& c) {
			std::stringstream colour_stream;
			std::string next, colour_str;

			// read in the rule name
			std::getline(input, c.m_name, ':');

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
			utility::strutil::removeAll(colour_str, std::string(" "));								// remove spaces.
			
			if (!colour_str.empty()) {
				colour_stream.str(colour_str);
				
				while (colour_stream.good()) {
					std::getline(colour_stream, next, ',');
					c.m_before.push_back(ClassifiedImage::getColourFromName(next));
				}
			}

			// ignore the rest of the line
			input.ignore(128, '\n');

			//middle
			//reset colour list
			c.m_middle.clear();
			
			// read in the middle: (min, max)
			input.ignore(30, '(');
			input >> c.m_middle_min;
			input.ignore(10, ',');
			input >> c.m_middle_max;
			input.ignore(10, ')');

			input.ignore(10, '[');

			//get colour list
			std::getline(input, colour_str, ']');
			utility::strutil::removeAll(colour_str, std::string(" "));								// remove spaces.
			
			if (!colour_str.empty()) {
				colour_stream.clear();
				colour_stream.str(colour_str);
				
				while (colour_stream.good()) {
					std::getline(colour_stream, next, ',');
					c.m_middle.push_back(ClassifiedImage::getColourFromName(next));
				}
			}
			
			// ignore the rest of the line
			input.ignore(128, '\n');

			//AFTER
			//reset colour list
			c.m_after.clear();
			
			// read in the after: (min, max)
			input.ignore(30, '(');
			input >> c.m_after_min;
			input.ignore(10, ',');
			input >> c.m_after_max;
			input.ignore(10, ')');

			input.ignore(10, '[');

			//get colour list
			std::getline(input, colour_str, ']');
			utility::strutil::removeAll(colour_str, std::string(" "));								// remove spaces.
			
			if (!colour_str.empty()) {
				colour_stream.clear();
				colour_stream.str(colour_str);
				
				while (colour_stream.good()) {
					std::getline(colour_stream, next, ',');
					c.m_after.push_back(ClassifiedImage::getColourFromName(next));
				}
			}

			// ignore the rest of the line
			input.ignore(128, '\n');
			
			//REPLACEMENT
			//get method
			input.ignore(20, ':');
			std::getline(input, colour_str, '/');
			utility::strutil::removeAll(colour_str, std::string(" "));								// remove spaces.
			
			if (!colour_str.empty()) {
				c.m_method = c.getMethodFromName(colour_str);
			}
			
			// ignore the rest of the line - potentially holds comment
			input.ignore(128, '\n');
			
			//force eofbit in the case of last rule
			input.peek();

			return input;
		}

		/*! @brief Stream extraction operator for a vector of ColourReplacementRule.
		 *  @relates ColourReplacementRule
		 */
		std::istream& operator>> (std::istream& input, std::vector<ColourReplacementRule>& v) {
			ColourReplacementRule temp;
			v.clear();
			
			while(input.good()) {
				input >> temp;
				v.push_back(temp);
			}

			return input;
		}

	}
}