#include "linedetector.h"
#include "boost/foreach.hpp"
#include "Tools/Math/LSFittedLine.h"

LineDetector::LineDetector() {}

LineDetector::~LineDetector() {}

/// @note this merges based on the first line, so ordering them is important
vector<pair<LSFittedLine, LSFittedLine> > LineDetector::mergeColinear(vector<pair<LSFittedLine, LSFittedLine> > lines,
                                                                      double angle_threshold, double distance_threshold) const
{
    //O(l^2)  -  l=number of lines
    // Compares all lines and merges based on the angle between and the average distance between

    vector<pair<LSFittedLine, LSFittedLine> > finals; // this vector contains lines that have been merged or did not need to be.
    pair<LSFittedLine, LSFittedLine> current; // line currently being compared with the rest.

    while(!lines.empty()) {
        //get next line
        current = lines.back();
        lines.pop_back();

        vector<pair<LSFittedLine, LSFittedLine> >::iterator it = lines.begin();
        //go through all lines and find any that should be merged - merge them
        while(it < lines.end()) {
            if(current.first.getAngleBetween(it->first) <= angle_threshold &&
               current.first.averageDistanceBetween(it->first) <= distance_threshold) {
                current.first.joinLine(it->first);  //join the other line to current
                current.second.joinLine(it->second);  //join the other paired lines
                it = lines.erase(it);   //remove the other line
            }
            else {
                it++;
            }
        }
        //Now current should have been merged with any valid lines
        //push current to finals
        finals.push_back(current);
    }

    return finals;
}
