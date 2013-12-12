/*
 * This file is part of SegmentedRegion.
 *
 * SegmentedRegion is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SegmentedRegion is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SegmentedRegion.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SegmentedRegion.h"

namespace modules {
    namespace vision {

		SegmentedRegion::SegmentedRegion() {
		}

		SegmentedRegion::SegmentedRegion(const SegmentedRegion& other) {
			SegmentedRegion(other.m_segmentedScans, other.m_direction);
		}

		SegmentedRegion::SegmentedRegion(const std::vector<std::vector<ColourSegment>>& segmentedScans, ScanDirection direction) {
			set(segmentedScans, direction);
		}

		void SegmentedRegion::set(const std::vector<std::vector<ColourSegment>>& segmentedScans, ScanDirection direction) {
			m_segmentedScans = segmentedScans;								//vector assignment operator copies elements
			m_direction = direction;
		}

		const std::vector<std::vector<ColourSegment>>& SegmentedRegion::getSegments() const {
			return m_segmentedScans;
		} 

		size_t SegmentedRegion::getNumberOfScans() const {
			return m_segmentedScans.size();
		}

		SegmentedRegion::ScanDirection SegmentedRegion::getDirection() const {
			return m_direction;
		}
		
		bool SegmentedRegion::empty() const {
			return m_segmentedScans.empty();
		}		
	}
}
