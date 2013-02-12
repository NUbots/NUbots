#include "segmentfilter.h"

#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"

#include <fstream>
#include <boost/foreach.hpp>


SegmentFilter::SegmentFilter()
{
    loadReplacementRules(RULE_DIR + "ReplacementRules");
    loadTransitionRules(RULE_DIR + "TransitionRules");
}

double averageLength(const SegmentedRegion& scans, Colour colour) {
    const vector<vector<ColourSegment> >& segments = scans.getSegments();
    vector<vector<ColourSegment> >::const_iterator line_it;
    vector<ColourSegment>::const_iterator seg_it;
    double sum = 0,
           num = 0;
    //loop through each scan
    for(line_it = segments.begin(); line_it < segments.end(); line_it++) {
        //move down segments in triplets replacing the middle if necessary
        for(seg_it = line_it->begin(); seg_it < line_it->end(); seg_it++) {
            if(seg_it->getColour() == colour) {
                sum += seg_it->getLength();
                num++;
            }
        }
    }
    return sum/num;
}

void SegmentFilter::run() const
{
    #if VISION_FILTER_VERBOSITY > 1
        debug << "SegmentFilter::run() - Begin" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const SegmentedRegion& h_segments = vbb->getHorizontalSegmentedRegion();
    const SegmentedRegion& v_segments = vbb->getVerticalSegmentedRegion();
    SegmentedRegion h_filtered, v_filtered;
    map<COLOUR_CLASS, vector<ColourSegment> > h_result, v_result;
    
    if(PREFILTER_ON) {

        preFilter(h_segments, h_filtered);
        preFilter(v_segments, v_filtered);
        vbb->setHorizontalFilteredSegments(h_filtered.m_segmented_scans);
        vbb->setVerticalFilteredSegments(v_filtered.m_segmented_scans);

        filter(h_filtered, h_result);
        filter(v_filtered, v_result);

        //count segment length
//        cout << averageLength(h_segments, yellow) << " ";
//        cout << averageLength(v_segments, yellow) << " ";
//        cout << averageLength(h_filtered, yellow) << " ";
//        cout << averageLength(v_filtered, yellow) << endl;
//        cout << averageLength(h_segments, green) << " ";
//        cout << averageLength(v_segments, green) << " ";
//        cout << averageLength(h_filtered, green) << " ";
//        cout << averageLength(v_filtered, green) << endl;
    }
    else {
        //Vision problem should occur in here:
        filter(h_segments, h_result);
        filter(v_segments, v_result);
    }
    
#if VISION_FILTER_VERBOSITY > 1
    ofstream outfile("1.txt");
    outfile << h_segments.getSegments();
    outfile.close();
    outfile.open("1f.txt");
    outfile << h_filtered.getSegments();
    outfile.close();
    outfile.open("2.txt");
    outfile << v_segments.getSegments();
    outfile.close();
    outfile.open("2f.txt");
    outfile << v_filtered.getSegments();
    outfile.close();
#endif
    //push results to BB
    vbb->setHorizontalTransitionsMap(h_result);
    vbb->setVerticalTransitionsMap(v_result);
}

void SegmentFilter::preFilter(const SegmentedRegion &scans, SegmentedRegion &result) const
{
    const vector<vector<ColourSegment> >& segments = scans.getSegments();
    vector<vector<ColourSegment> >& final_segments = result.m_segmented_scans;
    vector<ColourSegment> line;
    
    vector<vector<ColourSegment> >::const_iterator line_it;
    vector<ColourSegment>::const_iterator before_it, middle_it, after_it;
    ScanDirection dir = scans.getDirection();
    
    result.m_direction = dir;

    final_segments.clear();
    
    //loop through each scan
    for(line_it = segments.begin(); line_it < segments.end(); line_it++) {
        if(line_it->size() >= 3) {
            //move down segments in triplets replacing the middle if necessary
            before_it = line_it->begin();
            middle_it = before_it+1;
            line.clear();
            line.push_back(*before_it);         //add the first segment
            for(after_it = before_it+2; after_it < line_it->end(); after_it++) {
                applyReplacements(*before_it, *middle_it, *after_it, line, dir);
                before_it = middle_it;
                middle_it = after_it;
            }
            line.push_back(line_it->back());    //add the last segment
            joinMatchingSegments(line);         //merge any now matching segments
            final_segments.push_back(line);
        }
        else {
            //push the unfiltered line into the result as it is too small to filter
            line.assign(line_it->begin(), line_it->end());
            final_segments.push_back(line);
        }
    }
}

void SegmentFilter::filter(const SegmentedRegion &scans, map<COLOUR_CLASS, vector<ColourSegment> > &result) const
{
    switch(scans.getDirection()) {
    case VERTICAL:
        BOOST_FOREACH(const ColourTransitionRule& rule, rules_v) {
            vector<ColourSegment>& segments = result[rule.getColourClass()];
            checkRuleAgainstRegion(scans, rule, segments);
        }
        break;
    case HORIZONTAL:
        BOOST_FOREACH(const ColourTransitionRule& rule, rules_h) {
            vector<ColourSegment>& segments = result[rule.getColourClass()];
            checkRuleAgainstRegion(scans, rule, segments);
        }
        break;
    default:
        errorlog << "SegmentFilter::filter - invalid direction";
        return;
    }   
}

void SegmentFilter::checkRuleAgainstRegion(const SegmentedRegion &scans, const ColourTransitionRule &rule, vector<ColourSegment> &matches) const
{
    const vector<vector<ColourSegment> >& segments = scans.getSegments();
    vector<ColourSegment>::const_iterator it;
    
    //loop through each scan
    BOOST_FOREACH(const vector<ColourSegment> vs, segments) {
        //move down segments in scan pairwise
        it = vs.begin();
        //first check start pair alone
        if(rule.match(ColourTransitionRule::nomatch, *it, *(it+1))) {
            matches.push_back(*it);
        }
        //then check the rest in triplets
        while(it < vs.end()-1) {
            if(rule.match(*(it-1), *it, *(it+1))) {
                matches.push_back(*it);
            }
            it++;
        }
        //lastly check final pair alone
        if(rule.match(*(it-1), *it, ColourTransitionRule::nomatch)) {
            matches.push_back(*it);
        }
    }
}

void SegmentFilter::applyReplacements(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after, vector<ColourSegment>& replacements, ScanDirection dir) const
{
    vector<ColourReplacementRule>::const_iterator rules_it, begin, end;
    ColourSegment temp_seg;
    
    switch(dir) {
    case VERTICAL:
        begin = replacement_rules_v.begin();
        end = replacement_rules_v.end();
        break;
    case HORIZONTAL:
        begin = replacement_rules_h.begin();
        end = replacement_rules_h.end();
        break;
    default:
        errorlog << "SegmentFilter::applyReplacements - invalid direction" << endl;
        return;
    }    
    
    temp_seg = middle;
    
    for(rules_it = begin; rules_it < end; rules_it++) {
        if(rules_it->match(before, middle, after)) {
            //replace middle using replacement method
            switch(rules_it->getMethod()) {
            case ColourReplacementRule::BEFORE:
                temp_seg.setColour(before.getColour());
                replacements.push_back(temp_seg);
                break;
            case ColourReplacementRule::AFTER:
                temp_seg.setColour(after.getColour());
                replacements.push_back(temp_seg);
                break;
            case ColourReplacementRule::SPLIT:
            {
                //generate two new segments matching each end and push them both back
                Vector2<double> start_pt = temp_seg.getStart(),
                                end_pt   = temp_seg.getEnd(),
                                mid_pt   = (start_pt + end_pt) * 0.5;
                temp_seg.set(start_pt, mid_pt, before.getColour());
                replacements.push_back(temp_seg);
                temp_seg.set(mid_pt, end_pt, after.getColour());
                replacements.push_back(temp_seg);
                break;
            }
            case ColourReplacementRule::INVALID:
                errorlog << "SegmentFilter::applyReplacements - invalid replacement rule" << endl;
                replacements.push_back(middle);
                break;
            }
            return; //replacements found so exit
        }
    }
    
    replacements.push_back(middle); //no replacement so keep middle
}

void SegmentFilter::joinMatchingSegments(vector<ColourSegment> &line) const
{
    vector<ColourSegment>::iterator before_it, after_it;
    before_it = line.begin();
    after_it=before_it+1;
    while(after_it<line.end()) {
        if(before_it->getColour() == after_it->getColour()) {
            before_it->join(*after_it);
            after_it = line.erase(after_it);
            before_it = after_it-1;
        }
        else {
            after_it++;
            before_it++;
        }
    }
}

void SegmentFilter::loadTransitionRules(string filename)
{
    //load the horizontal rules
    string temp_filename = filename + "_h.txt";
    ifstream input(temp_filename.c_str());
    
    if(input.good()) {
        input >> rules_h;
    }
    else {
        debug << "SegmentFilter::loadTransitionRules - failed to read from " << temp_filename << endl;
    }
    input.close();


    //load the vertical rules
    temp_filename = filename + "_v.txt";
    input.open(temp_filename.c_str());

    if(input.good()) {
        input >> rules_v;
    }
    else {
        debug << "SegmentFilter::loadTransitionRules - failed to read from " << temp_filename << endl;
    }
    input.close();

    if(rules_h.size()  == 0 || rules_v.size() == 0){
        cout <<"=========================WARNING=========================\n"
             << "SegmentFilter::loadTransitionRules - " << filename
             <<"_v.txt or _h.txt empty!\n \n" << "The robot may exhibit blindness."
             <<"=========================WARNING=========================\n" << endl;
    }
        //DEBUG
#if VISION_FILTER_VERBOSITY > 0
    debug << "SegmentFilter::loadTransitionRules()" << endl;
    debug << "rules_h (" << rules_h.size() << ")\n" << rules_h;
    debug << "rules_v (" << rules_v.size() << ")\n" << rules_v;
#endif
}

void SegmentFilter::loadReplacementRules(string filename)
{
    //load the horizontal rules
    string temp_filename = filename + "_h.txt";
    ifstream input(temp_filename.c_str());

    if(input.good()) {
        input >> replacement_rules_h;
    }
    else {
        debug << "SegmentFilter::loadReplacementRules - failed to read from " << temp_filename << endl;
    }
    input.close();

    
    //load the vertical rules
    temp_filename = filename + "_v.txt";
    input.open(temp_filename.c_str());

    if(input.good()) {
        input >> replacement_rules_v;
    }
    else {
        debug << "SegmentFilter::loadReplacementRules - failed to read from " << temp_filename << endl;
    }
    input.close();
    
    //DEBUG
#if VISION_FILTER_VERBOSITY > 0
    debug << "SegmentFilter::loadReplacementRules()" << endl;
    debug << "replacement_rules_h (" << replacement_rules_h.size() << ")\n" << replacement_rules_h;
    debug << "replacement_rules_v (" << replacement_rules_v.size() << ")\n" << replacement_rules_v;
#endif
}
