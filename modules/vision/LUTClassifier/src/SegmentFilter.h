/*
 * Note that the segment filter no longer supports vertical and horizontal rules
 * This file is part of SegmentFilter.
 *
 * SegmentFilter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SegmentFilter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SegmentFilter.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_SEGMENTFILTER_H
#define MODULES_VISION_SEGMENTFILTER_H

#include <nuclear> 
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <armadillo>

#include "utility/vision/ColourSegment.h"
#include "messages/vision/ClassifiedImage.h"

#include "ColourReplacementRule.h"
#include "ColourTransitionRule.h"

namespace modules {
    namespace vision {
	
		class SegmentFilter {
		public:
			static const bool PREFILTER_ON = true;
			SegmentFilter();
			
			/**
			  @brief runs the segment filter over the horizontal and vertical segments std::lists.
			  This matches pairs of segments to preloaded transition rules and stores matching results
			  as transitions back on the blackboard. This method also calls some smoothing prefilters on the std::lists
			  which are also set by preloaded rules.
			  */
			std::make_unique<messages::vision::ClassifiedImage> classifyImage(const utility::vision::SegmentedRegion& horizontalSegments, 
																				const utility::vision::SegmentedRegion& verticalSegments) const;
				
			void clearRules();
			void addTransitionRule(const ColourTransitionRule& rule);
			void addReplacementRule(const ColourReplacementRule& rule);

		private:
			/**
			  @brief runs the segment prefilter rules over a segment std::list.
			  @param scans the std::lists of segments.
			  @param result a smoothed result.
			  */
			void preFilter(const utility::vision::SegmentedRegion& scans, utility::vision::SegmentedRegion& result) const;
			
			/**
			  @brief runs the transition rules over a segment std::list.
			  @param scans the std::lists of segments - smoothed or unsmoothed.
			  @param result std::vectors of transition rule matches and the field object ids they map to.
			  */
			void filter(const utility::vision::SegmentedRegion& scans, std::map<utility::vision::COLOUR_CLASS, std::vector<utility::vision::ColourSegment>>& result) const;
		
			/**
			  @brief Applies a single rule to a segmented region.
			  @param scans the std::lists of segments - smoothed or unsmoothed.
			  @param rule The transition rule to apply.
			  @param matches the resulting std::list of transitions.
			  */
			void checkRuleAgainstRegion(const utility::vision::SegmentedRegion& scans, const ColourTransitionRule& rule, std::vector<utility::vision::ColourSegment>& matches) const;
			
			/**
			  @brief Applies a replacement rule to a triplet of segments.
			  @param before the first segment.
			  @param middle the second segment.
			  @param after the last segment.
			  @param replacement a reference to a std::vector of segments that should replace the middle segment.
			  @param dir the scan direction (vertical or horizontal).
			  */
			void applyReplacements(const utility::vision::ColourSegment& before, 
									const utility::vision::ColourSegment& middle, 
									const utility::vision::ColourSegment& after, std::vector<utility::vision::ColourSegment>& replacement, 
									utility::vision::ScanDirection dir) const;
				
			/**
			  @brief Joins any adjacent segments that are the same colour.
			  @param line the std::list of segments.
			  */
			void joinMatchingSegments(std::vector<utility::vision::ColourSegment>& line) const;

		
		private:
			std::vector<ColourReplacementRule> m_horizontalReplacementRules;	//! @variable The std::list of horizontal replacement rules
			std::vector<ColourReplacementRule> m_verticalReplacementRules;		//! @variable The std::list of vertical replacement rules
			std::vector<ColourTransitionRule> m_horizontalRules;				//! @variable The std::list of horizontal transition rules
			std::vector<ColourTransitionRule> m_verticalRules;					//! @variable The std::list of vertical transition rules
    	};    	
		
    }  // vision
}  // modules

#endif  // MODULES_VISION_SEGMENTFILTER_H

