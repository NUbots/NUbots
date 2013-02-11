#include "ransacgoal.h"
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

RANSACGoal::RANSACGoal()
{
}

bool RANSACGoal::regenerate(const vector<ColourSegment>& segments)
{
    if(segments.size() == 2) {
        setLineFromPoints(segments[0].getCentre(), segments[1].getCentre());
        return true;
    }
    else {
        return false;
    }
}

double RANSACGoal::calculateError(ColourSegment c) const
{
    return getLinePointDistance(c.getCentre());
}

void RANSACGoal::fit(const vector<ColourSegment> &segments)
{
    accumulator_set<double, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(const ColourSegment& seg, segments) {
        addPoint(seg.getCentre());
        acc(seg.getLength());
    }

    width_mean = mean(acc);
    width_stddev = sqrt(variance(acc));
}
