#include "ransac.h"
#include <limits>
#include <stdlib.h>
#include <boost/foreach.hpp>

namespace RANSAC
{
    std::vector<LSFittedLine> findMultipleLines(const std::vector<LinePoint>& line_points, float e, unsigned int n, unsigned int k, unsigned int max_iterations)
    {
        float variance;
        Line line;
        LSFittedLine fitted_line;
        std::vector<LSFittedLine> results;
        std::vector<LinePoint> consensus;
        std::vector<LinePoint> remainder;

        //run first iterations
        bool line_found = findLine(line_points, line, consensus, remainder, variance, e, n, k);
        int i=0;
        while(line_found && i<max_iterations) {
            fitted_line.clearPoints();
            fitted_line.addPoints(consensus);
            results.push_back(fitted_line);
            line_found = findLine(remainder, line, consensus, remainder, variance, e, n, k);
            i++;
        }

        return results;
    }

//    bool findLine(std::vector<LinePoint> points, Line& result, std::vector<LinePoint>& consensus, std::vector<LinePoint>& remainder, float& variance, float e, unsigned int n, unsigned int k)
//    {
//        if (points.size() < n) {
//            return false;
//        }

//        // error of best line found so far
//        float minerr = std::numeric_limits<float>::max();
//        // arrays for storing concensus sets
//        bool c1[points.size()];
//        bool c2[points.size()];

//        bool* best_concensus;
//        bool* cur_concensus;
//        best_concensus = c1;
//        consensus.clear();

//        for (unsigned int i = 0; i < k; ++i) {
//          // randomly select 2 distinct points
//            Line l = generateRandomLine(points);
//            variance = 0;
//            // figure out the variance (sum of distances of points from the line)
//            // could use dist() here, but since the denominator is consistent, we
//            // save time and implement it again here.
//    //      float denom = l.getNormaliser();
//    //      float newe = m_e*denom;
//            float newe = e*l.getNormaliser();
//            unsigned int concensus_size = 0;

//            if (c1 == best_concensus)
//               cur_concensus = c2;
//            else
//               cur_concensus = c1;

//            for (unsigned int i = 0; i < points.size(); i++) {
//                float dist = l.getLinePointDistance(points.at(i));
//                if (dist < newe) {
//                    variance += dist;
//                    concensus_size++;
//                    cur_concensus[i] = true;
//                }
//                else {
//                    cur_concensus[i] = false;
//                }
//            }
//    //      variance /= denom;
//            static float scaling_factor = 0.2;
//            variance = scaling_factor*variance - concensus_size;
//            if (variance < minerr && concensus_size >= n) {
//                minerr = variance;
//                result = l;
//                best_concensus = cur_concensus;
//            }
//        }
//        variance = variance/(points.size()*e);

//        consensus.clear();
//        remainder.clear();

//        if (minerr < std::numeric_limits<float>::max()) {
//            for(unsigned int i=0; i<points.size(); i++) {
//                if(best_concensus[i])
//                    consensus.push_back(points.at(i));
//                else {
//                    remainder.push_back(points.at(i));
//                }
//            }
//            return true;
//        }
//        else {
//            return false;
//        }
//    }

    bool findLine(std::vector<LinePoint> points, Line& result, std::vector<LinePoint>& consensus, std::vector<LinePoint>& remainder, float& variance, float e, unsigned int n, unsigned int k)
    {
        if (points.size() < n || n==0) {
            return false;
        }

        // error of best line found so far
        //float minerr = std::numeric_limits<float>::max();
        // arrays for storing concensus sets
        bool c1[points.size()];
        bool c2[points.size()];

        bool* best_concensus;
        bool* cur_concensus;
        best_concensus = c1;
        unsigned int best_concensus_size = 0;
        float best_consensus_variance = std::numeric_limits<float>::max();
        float cur_variance;

        for (unsigned int i = 0; i < k; ++i) {
            // randomly select 2 distinct points
            Line l = generateRandomLine(points);
            cur_variance = 0;

            unsigned int concensus_size = 0;

            if (c1 == best_concensus)
               cur_concensus = c2;
            else
               cur_concensus = c1;

            for (unsigned int i = 0; i < points.size(); i++) {
                float dist = l.getLinePointDistance(points.at(i));
                if (dist < e) {
                    cur_variance += dist;
                    concensus_size++;
                    cur_concensus[i] = true;
                }
                else {
                    cur_concensus[i] = false;
                }
            }
//            static float scaling_factor = 0.2;
//            variance = scaling_factor*variance - concensus_size;
//            if (variance < minerr && concensus_size >= n) {
//                minerr = variance;
//                result = l;
//                best_concensus = cur_concensus;
//            }
            if(concensus_size > best_concensus_size) {
                best_concensus_size = concensus_size;
                best_consensus_variance = cur_variance/concensus_size;
                result = l;
                best_concensus = cur_concensus;
            }
        }
        //variance = variance/(points.size()*e);
        variance = best_consensus_variance;

        consensus.clear();
        remainder.clear();

        if (best_concensus_size >= n) {
            for(unsigned int i=0; i<points.size(); i++) {
                if(best_concensus[i])
                    consensus.push_back(points.at(i));
                else {
                    remainder.push_back(points.at(i));
                }
            }
            return true;
        }
        else {
            return false;
        }
    }

    Line generateRandomLine(const std::vector<LinePoint>& points)
    {
        if(points.size() > 1) {
            LinePoint p1, p2;
            p1 = points.at(rand() % points.size());
            do {
                p2 = points.at(rand() % points.size());
            }
            while (p1 == p2);
            return Line(p1, p2);
        }
        else {
            return Line();
        }
    }
}

