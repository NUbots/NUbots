#include "linedetector.h"
#include "Vision/visionblackboard.h"
#include "boost/foreach.hpp"
#include "Tools/Math/LSFittedLine.h"

LineDetector::LineDetector()
{
}

void LineDetector::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();

    vector<Transition> v_trans = vbb->getVerticalTransitions(VisionFieldObject::LINE);  //get transitions associated with lines
    vector<Transition> h_trans = vbb->getHorizontalTransitions(VisionFieldObject::LINE);
    vector<LSFittedLine*> lines;
    vector<LSFittedLine> result;
    vector<LinePoint*> points;

    points = getPointsFromTransitions(h_trans, v_trans);

    points = pointsUnderGreenHorizon(points, vbb->getGreenHorizon());

    lines = m_SAM.run(points, true);

    BOOST_FOREACH(LSFittedLine* l, lines) {
        cout << l->getA() << "x + " << l->getB() << "y = " << l->getC() << " - r2tls: " << l->getr2tls() << " - msd: " << l->getMSD() << " - #points: " << l->numPoints;

        result.push_back(*l);
    }

    vbb->addLines(result);
}

vector<LinePoint*> LineDetector::getPointsFromTransitions(const vector<Transition>& h_trans, const vector<Transition>& v_trans)
{
    vector<LinePoint*> points;
    //Old, just uses transitions
//    BOOST_FOREACH(Transition t, h_trans) {
//        points.push_back(new LinePoint(t.getLocation().x, t.getLocation().y));
//    }
//    BOOST_FOREACH(Transition t, v_trans) {
//        points.push_back(new LinePoint(t.getLocation().x, t.getLocation().y));
//    }

    //new - finds pairs of transitions and finds their centrepoints
    // a bit of a hack, assumes there are always pairs and that the transitions are in order.
    int av_x, av_y;
    vector<Transition>::const_iterator t_it;
    if(h_trans.size()%2 == 0) {
        t_it = h_trans.begin();
        while(t_it < h_trans.end()) {
            av_x = (t_it->getLocation().x + (t_it+1)->getLocation().x)*0.5;
            av_y = (t_it->getLocation().y + (t_it+1)->getLocation().y)*0.5;
            cout << "(" << t_it->getLocation().x << "," << t_it->getLocation().y << ") (" << (t_it+1)->getLocation().x << "," << (t_it+1)->getLocation().y << ") (" << av_x << "," << av_y << ")" << std::endl;
            points.push_back(new LinePoint(av_x ,av_y));
            t_it += 2;
        }
    }
    else {
        errorlog << "LineDetector::getPointsFromTransitions: unpaired horizontal line transitions!" << std::endl;
    }
    cout << "V\n";
    if(v_trans.size()%2 == 0) {
        t_it = v_trans.begin();
        while(t_it < v_trans.end()) {
            av_x = (t_it->getLocation().x + (t_it+1)->getLocation().x)*0.5;
            av_y = (t_it->getLocation().y + (t_it+1)->getLocation().y)*0.5;
            cout << "(" << t_it->getLocation().x << "," << t_it->getLocation().y << ") (" << (t_it+1)->getLocation().x << "," << (t_it+1)->getLocation().y << ") (" << av_x << "," << av_y << ")" << std::endl;
            points.push_back(new LinePoint(av_x ,av_y));
            t_it += 2;
        }
    }
    else {
        errorlog << "LineDetector::getPointsFromTransitions: unpaired vertical line transitions!" << std::endl;
    }

    return points;
}

vector<LinePoint*> LineDetector::pointsUnderGreenHorizon(const vector<LinePoint*> points, const GreenHorizon& gh)
{
    vector<LinePoint*> under;
    BOOST_FOREACH(LinePoint* p, points) {
        if(gh.isBelowHorizon(PointType(p->x, p->y))) {
            under.push_back(p);
        }
    }
    return under;
}
