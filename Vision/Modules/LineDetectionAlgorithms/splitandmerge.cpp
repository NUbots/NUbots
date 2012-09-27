#include "splitandmerge.h"
//#include "Tools/Profiling/Profiler.h"
#include "debug.h"
#include "Vision/visionconstants.h"
#include "Tools/Math/General.h"
//#include <QDebug>

#include <boost/foreach.hpp>

SplitAndMerge::SplitAndMerge()
{
}

SplitAndMerge::~SplitAndMerge()
{
}

//CLUSTERS
//void SplitAndMerge::splitAndmergeClusters(vector<LSFittedLine>& lines, vector< vector<LinePoint> >& clusters, vector<LinePoint> leftover, Vision* vision, LineDetection* linedetector, bool clearsmall, bool cleardirty, bool noise) {
//    //Performs split-and-merge algorithm with input consisting of a set of point clusters
//    // and a set of unclustered points, putting the resulting lines into a reference
//    // passed vector

//    //Profiler prof("SplitAndMerge");
//    noFieldLines = 0;
//    noisePoints.clear();

//    //For each cluster..

//    //prof.start();
//    for(unsigned int i=0; i<clusters.size(); i++) {
//        //perform split - splitLS() checks for appropriate size, so that need not be done here
//        //noisePoints.clear();
//        noisePoints1.clear();
//        noisePoints2.clear();
//        splitIterative(lines, clusters[i]);
//        splitNoiseLS12(lines);
//    }
//    //prof.split("Split Clusters");

//    //Then split leftover
//    splitIterative(lines, leftover);
//    //prof.split("Leftover");

//    //Then noise
//    if(noise) {
//        splitNoiseLS(lines);
//    }
//    //prof.split("Noise");

//    //Do Centre Circle fitting before merge - To do later

//    //Then Merge
//    convertLinesEndPoints(lines, vision, linedetector);

//    merge(lines, linedetector, vision);
//    //prof.split("Merge");

//    //Then clear unwanted lines
//    if(clearsmall) {
//        clearSmallLines(lines);
//    }
//    if(cleardirty) {
//        clearDirtyLines(lines);
//    }
//    //prof.split("Clear Unwanted");
//    //debug << prof;


//    noisePoints.clear();
//}

vector<LSFittedLine> SplitAndMerge::run(vector<LinePoint>& points, bool noise) {
    //Performs split-and-merge algorithm with input consisting of a set of point clusters
    // and a set of unclustered points, putting the resulting lines into a reference
    // passed vector
    //Import parameters from constants file
    SPLIT_DISTANCE = VisionConstants::SAM_SPLIT_DISTANCE;
    MIN_POINTS_OVER = VisionConstants::SAM_MIN_POINTS_OVER;
    MAX_ANGLE_DIFF_TO_MERGE = VisionConstants::SAM_MAX_ANGLE_DIFF_TO_MERGE;
    MAX_DISTANCE_TO_MERGE = VisionConstants::SAM_MAX_DISTANCE_TO_MERGE;
    MIN_POINTS_TO_LINE = VisionConstants::SAM_MIN_POINTS_TO_LINE;
    MIN_POINTS_TO_LINE_FINAL = VisionConstants::SAM_MIN_POINTS_TO_LINE_FINAL;
    MIN_LINE_R2_FIT = VisionConstants::SAM_MIN_LINE_R2_FIT;
    MAX_LINE_MSD = VisionConstants::SAM_MAX_LINE_MSD;
    MAX_POINTS = VisionConstants::SAM_MAX_POINTS;
    MAX_LINES = VisionConstants::SAM_MAX_LINES;
    SPLIT_NOISE_ITERATIONS = VisionConstants::SAM_SPLIT_NOISE_ITERATIONS;
    CLEAR_SMALL = VisionConstants::SAM_CLEAR_SMALL;
    CLEAR_DIRTY = VisionConstants::SAM_CLEAR_DIRTY;

    vector<LSFittedLine> lines;
    noisePoints.clear();

    //splitIterative(lines, points);
    split(lines, points);

    //Then noise
    if(noise) {
        splitNoise(lines);
    }

    //Do Centre Circle fitting before merge - To do later

    //Then Merge
    //convertLinesEndPoints(lines, vision, linedetector);
    merge(lines);

    //Then clear unwanted lines
    if(CLEAR_SMALL) {
        clearSmallLines(lines);
    }
    if(CLEAR_DIRTY) {
        clearDirtyLines(lines);
    }

    return lines;
}



void SplitAndMerge::split(vector<LSFittedLine>& lines, vector<LinePoint>& points) {
    // Recursive split algorithm - not used

    //Assumes:
    //	- constant detirmined for limit - MIN_POINTS_OVER
    //	- constant for min splitting distance - SAM_THRESHOLD

    //Boundary Conds
    if(lines.size() >= MAX_LINES) {
        addToNoise(points);
        return;
    }
    if(points.size() < MIN_POINTS_TO_LINE) {
        //add points to noise
        addToNoise(points);
        return;
    }

    //temp variables
    unsigned int points_over = 0; //how many points are further from the line than SAM_THRESHOLD
    int greatest_point = 0; //which point in vector is the furthest

    //generate new LSFittedLine
    LSFittedLine line;
    line.addPoints(points);

    //check for points over threshold
    findPointsOver(line, points_over, greatest_point);

    //if num points over threshold > limit -> split at greatest distance point.
    if(points_over >= MIN_POINTS_OVER) {
        //there are enough points distant to justify a split
        vector<LinePoint> left;    //holder vectors
        vector<LinePoint> right;
        if(separate(left, right, points[greatest_point], line)) {
            //split was valid - recursively split new lines
            split(lines, left);
            split(lines, right);
        }
        else {
            //remove furthest point and retry
            vector<LinePoint> newlist = points;
            addToNoise(newlist[greatest_point]);
            newlist.erase(newlist.begin() + greatest_point);
            split(lines, newlist);
        }

    }
    else if(points_over > 0) {
        //not enough points over to split so remove point as noisy, and regen line
        if(points.size() > MIN_POINTS_TO_LINE_FINAL) {
            //removal of a point will still leave enough to form a reasonable line
            vector<LinePoint> newlist = points;
            addToNoise(newlist[greatest_point]);
            for(unsigned int i=greatest_point; i<newlist.size()-1; i++) {
                newlist[i] = newlist[i+1];
            }
            newlist.pop_back();
            line.clearPoints();
            generateLine(line, newlist);
            lines.push_back(line);
        }
        else {
            //NOT SURE ??
            //Add points to noise and delete line
            addToNoise(points);
        }
    }
    else {
        //no points over, just push line
        lines.push_back(line);
    }
}

//void SplitAndMerge::splitIterative(vector<LSFittedLine>& lines, vector<LinePoint>& points) {
//    //Iterative split algorithm, uses a stack of lines and iterates over it, splitting
//    //each line or adding it to a final list.

//    //Boundary Conds
//    if(lines.size() >= MAX_LINES) {
//        return;
//    }
//    if(points.size() < MIN_POINTS_TO_LINE) {
//        //add points to noise
//        for(unsigned int i=0; i<points.size(); i++)
//            addToNoise(points[i]);
//        return;
//    }

//    //Locals
//    vector<LSFittedLine> stack;
//    stack.clear();
//    int furthest_point;
//    int points_over;
//    vector<LinePoint> left, right;

//    LSFittedLine* tempLine = new LSFittedLine();
//    //generate first line
//    generateLine(*tempLine, points);
//    //push line onto stack
//    stack.push_back(tempLine);

//    //Begin iteration
//    while(!stack.empty() && lines.size() + stack.size() < MAX_LINES) {
//        //Pop the top line off and split it if warranted
//        //if not, slap it on the end of lines and go again
//        //until stack is empty or maximum lines reached

//        //Clear left and right
//        left.clear();
//        right.clear();

//        //pop top line
//        tempLine = stack.back();
//        stack.pop_back();

//        //check for points over threshold
//        findFurthestPoint(*tempLine, points_over, furthest_point);
//        //qDebug() << points_over << furthest_point;
//        //Options
//        if((unsigned int)points_over >= MIN_POINTS_OVER) {
//            //See if separation is an option
//            //qDebug() << "going to seperate the line: " << furthest_point << (tempLine->getPoints()[furthest_point])->x << (tempLine->getPoints()[furthest_point])->y;
//            if(separate(left, right, tempLine->getPoints()[furthest_point], *tempLine)) {
//                //qDebug() << "separating worked";
//                //clear old line
//                if(!useTripleSplit)
//                {
//                    tempLine->getPoints().clear();
//                    delete tempLine;
//                    //check if left is big enough
//                    if(left.size() >= MIN_POINTS_TO_LINE) {
//                        //generate line and push it to stack
//                        tempLine = new LSFittedLine();
//                        generateLine(*tempLine, left);
//                        stack.push_back(tempLine);
//                    }
//                    else {
//                        //throw left points to noise
//                        for(unsigned int i=0; i<left.size(); i++)
//                            addToNoise(left[i]);
//                        left.clear();
//                    }
//                    //check if right is big enough
//                    if(right.size() >= MIN_POINTS_TO_LINE) {
//                        //generate line and push it to stack
//                        tempLine = new LSFittedLine();
//                        generateLine(*tempLine, right);
//                        stack.push_back(tempLine);
//                    }
//                    else {
//                        //throw right points to noise
//                        for(unsigned int i=0; i<right.size(); i++)
//                            addToNoise(right[i]);
//                        right.clear();
//                    }
//                }
//                else //triple split
//                {
//                    //qDebug() << "Using the Tripple Split";
//                    tempLine->getPoints().clear();
//                    delete tempLine;
//                    //check if left is big enough
//                    if(centre.size() >= MIN_POINTS_TO_LINE) {
//                        //generate line and push it to stack
//                        tempLine = new LSFittedLine();
//                        generateLine(*tempLine, centre);
//                        stack.push_back(tempLine);
//                    }
//                    else {
//                        //throw left points to noise
//                        for(unsigned int i=0; i<centre.size(); i++)
//                            addToNoise(centre[i]);
//                        centre.clear();
//                    }
//                    //check if right is big enough
//                    if(above.size() >= MIN_POINTS_TO_LINE) {
//                        //generate line and push it to stack
//                        tempLine = new LSFittedLine();
//                        generateLine(*tempLine, above);
//                        stack.push_back(tempLine);
//                    }
//                    else {
//                        //throw right points to noise
//                        for(unsigned int i=0; i<above.size(); i++)
//                            addToNoise(above[i]);
//                        above.clear();
//                    }
//                    if(below.size() >= MIN_POINTS_TO_LINE) {
//                        //generate line and push it to stack
//                        tempLine = new LSFittedLine();
//                        generateLine(*tempLine, below);
//                        stack.push_back(tempLine);
//                    }
//                    else {
//                        //throw right points to noise
//                        for(unsigned int i=0; i<below.size(); i++)
//                            addToNoise(below[i]);
//                        below.clear();
//                    }

//                }
//            } //if(separate())
//            else {
//                //Separation didn't work
//                //remove furthest point and push new line
//                vector<LinePoint> newlist = tempLine->getPoints();
//                //remove noisy point and update points list
//                addToNoise(newlist[furthest_point]);
//                newlist[furthest_point] = newlist[newlist.size() - 1];
//                newlist[newlist.size()-1] = 0;
//                newlist.pop_back();
//                //clear old line
//                tempLine->getPoints().clear();
//                delete tempLine;
//                tempLine = new LSFittedLine();
//                generateLine(*tempLine, newlist); //make new line
//                stack.push_back(tempLine); //push it back
//            }
//        } //if(points_over >= MIN_POINTS_OVER)
//        else if(points_over > 0){
//            //not enough points over to split - but there are points over
//            if(tempLine->getPoints().size() > MIN_POINTS_TO_LINE_FINAL) {
//                //i.e. removal of a point will still leave enough to form a reasonable line
//                //remove noisy point and regen line
//                vector<LinePoint> newlist = tempLine->getPoints();
//                //remove noisy point and update points list
//                addToNoise(newlist[furthest_point]);
//                newlist[furthest_point] = newlist[newlist.size() - 1];
//                newlist[newlist.size()-1] = 0;
//                newlist.pop_back();
//                tempLine->getPoints().clear();
//                delete tempLine; //clear memory of old line
//                tempLine = new LSFittedLine();
//                generateLine(*tempLine, newlist); //make new line
//                //qDebug() << "pushing line";
//                lines.push_back(tempLine); //push it back to finals
//                noFieldLines++;
//            }
//            else {
//                //Add points to noise and delete line
//                for(unsigned int i=0; i<tempLine->getPoints().size(); i++)
//                    addToNoise(tempLine->getPoints()[i]);
//                tempLine->getPoints().clear();
//                delete tempLine;
//            }
//        } //elseif(points_over > 0)
//        else {
//            //no points over, just push line to finals
//            //qDebug() << "pushing line";
//            lines.push_back(tempLine);
//            noFieldLines++;
//        }
//    } // While(!stack.empty() && noFieldLines + stack.size() <= MAX_LINES)


////qDebug() << stack.size();
//    //If MAX_LINES reached, but stack is not empty push stack lines to finals
//    if(noFieldLines + stack.size() == MAX_LINES) {
//        //qDebug() << "Maximum Lines reached in split";
//    }
//    while(!stack.empty()) {
//        lines.push_back(stack.back());
//        stack.pop_back();
//    }
//}

void SplitAndMerge::findPointsOver(LSFittedLine& line, unsigned int& points_over, int& furthest_point) {
    //this method finds the furthest point from a line and returns (via parameters)
    //the number of points over the SPLIT_DISTANCE threshold and the index of
    //the furthest point

    //temp variables
    double distance; 	//holder for calculated LinePointDistance
    double greatest_distance = 0.0; //saves recalculation of greatest point distance
    unsigned int current_point = 0;
    points_over = 0;
    furthest_point = -1;
    vector<LinePoint> points = line.getPoints();

    //check points for perp distance over threshold
    for(current_point = 0; current_point < points.size(); current_point++) {
        distance = line.getLinePointDistance(points[current_point]);
        //qDebug() <<current_point <<distance;

        if(distance > SPLIT_DISTANCE) {
            //potential splitting point
            points_over++; //increment points_over counter
            //check if greatest point
            if(distance > greatest_distance) {
                //new furthest point found
                greatest_distance = distance;
                furthest_point = current_point;
            }
        }
    }
    //qDebug() <<furthest_point <<greatest_distance;
}

void SplitAndMerge::splitNoise(vector<LSFittedLine>& lines) {
    //this method creates a copy of the noisePoints vector,
    //clears the current noisePoints vector and runs
    //the split algorithm on the copy

    if(noisePoints.size() >= MIN_POINTS_TO_LINE_FINAL) {
        vector<LinePoint> noiseCopy;

        noiseCopy = noisePoints;
        noisePoints.clear();
        //splitIterative(lines, noiseCopy);
        split(lines, noiseCopy);
    }
}

bool SplitAndMerge::separate(vector<LinePoint>& left, vector<LinePoint>& right, LinePoint& split_point, LSFittedLine& line) {
    /*splits a section of points around a splitting point by rotating and translating onto the line about the splitting point
     *Pre: left and right should be empty vectors
     *		points contains all the points to be split
     *		split_point is a valid point in points
     *		line is the LSFittedLine
     *Post: left contains all points with negative transformed x-vals
     *		right contains all points with non-negative transformed x-vals (note split_point will be in right)
     *      if left or right is empty, returns false indicating no actual split occurred
    */

    //temp holder vars
    double x_split = split_point.x;
    double y_split = split_point.y;
    vector<LinePoint> points = line.getPoints();

    left.push_back(split_point);    //splitting point should be included in both groups
    right.push_back(split_point);
    if(line.isHorizontal()) {
        //horizontal line - no rotation
        BOOST_FOREACH(LinePoint pt, points) {
            if(pt != split_point) {
                if(pt.x < x_split) //point is to the left
                    left.push_back(pt);
                else
                    right.push_back(pt);
            }
        }
    }
    else if(line.isVertical()) {
        //vertical line - 90 degree rotation
        BOOST_FOREACH(LinePoint pt, points) {
            if(pt != split_point) {
                if(pt.y < y_split) //point is to the left
                    left.push_back(pt);
                else
                    right.push_back(pt);
            }
        }
    }
    else {
        double xsplit = line.projectOnto(split_point).x;
        BOOST_FOREACH(LinePoint pt, points) {
            //check all points, calculate translated x coord
            //and place in appropriate vector
            if(pt != split_point) {
                if(line.projectOnto(pt).x < xsplit) {
                    //point is to the left
                    left.push_back(pt);
                }
                else {
                    right.push_back(pt);
                }
            }
        }
    }
    //if either left or right contains entire point set then there will be an
    //infinite loop


    //! WHAT WAS THE FOLLOWING FOR??
//    if((abs(above.size() - below.size()) < abs(left.size() - right.size())) && above.size() != 0 && below.size() !=0 && centre.size() !=0)
//    {
//        if((above.size() < points.size()) && (below.size() < points.size()) && (centre.size() < points.size()/2))
//            useTripleSplit = true;
//    }

    return (left.size() < points.size() && right.size() < points.size());
}


void SplitAndMerge::merge(vector<LSFittedLine>& lines) {
    //O(l^2)  -  l=number of lines (max 15)
    // Compares all lines and merges based on the return value of
    // shouldMergeLines(Line, Line) - edit that method not this one

    vector<LSFittedLine> finals; // this vector contains lines that have been merged or did not need to be.
    LSFittedLine current; // line currently being compared with the rest.


    while(!lines.empty()) {
        //get next line
        current = lines.back();
        lines.pop_back();

        vector<LSFittedLine>::iterator it = lines.begin();
        //go through all lines and find any that should be merged - merge them
        while(it < lines.end()) {
            if(shouldMergeLines(current, *it)) {
                //cout << *current << " & " << **it << " should merge: true" << std::endl;
                //join the lines
                current.joinLine(*it);
                //remove the considered line and update line list
                it = lines.erase(it);
            }
            else {
                //cout << *current << " & " << **it << " should merge: false" << std::endl;
                it++;
            }
        }
        //Now current should have been merged with any valid lines
        //push current to finals
        finals.push_back(current);
    }

    lines = finals;
}

void SplitAndMerge::generateLine(LSFittedLine& line, vector<LinePoint>& points) {
    //creates a Least Squared Fitted line

    line.clearPoints();
    line.addPoints(points);
}

//GENERIC

void SplitAndMerge::addToNoise(const LinePoint& point) {
    //NOT EFFICIENT
    //O(M) for every insertion - where M is the size of noisePoints
    BOOST_FOREACH(LinePoint pt, noisePoints) {
        if(pt == point)
            return;
    }
    //only occurs if there are not copies of the point in the noise list
    noisePoints.push_back(point);
}

void SplitAndMerge::addToNoise(const vector<LinePoint > &points) {
    BOOST_FOREACH(LinePoint pt, points) {
        addToNoise(pt);
    }
}

void SplitAndMerge::clearSmallLines(vector<LSFittedLine>& lines) {
    //removes any lines from the vector whose vector of
    //member points is too small

    vector<LSFittedLine>::iterator it = lines.begin();

    while(it < lines.end()) {
        if(it->numPoints < MIN_POINTS_TO_LINE_FINAL) {
            it = lines.erase(it);
        }
        else {
            it++;
        }
    }
}


void SplitAndMerge::clearDirtyLines(vector<LSFittedLine>& lines) {
    //removes any lines from the vector whose R^2 value is
    //less than MIN_LINE_R2_FIT
    vector<LSFittedLine>::iterator it = lines.begin();

    while(it < lines.end()) {
        if(it->getr2tls() < MIN_LINE_R2_FIT || it->getMSD() > MAX_LINE_MSD) {
            it = lines.erase(it);
        }
        else {
            it++;
        }
    }
}

bool SplitAndMerge::shouldMergeLines(const LSFittedLine& line1, const LSFittedLine& line2){
    //check lines have similar gradients by checking the angle between them (linear with rotation).
//    cout << "angle between: " << line1.getAngleBetween(line2) << std::endl;
//    cout << "line1 angle: " << abs(line1.getAngle()) << "  > pi/4? " << (abs(line1.getAngle()) > mathGeneral::PI*0.25) << std::endl;
//    cout << "x intercepts : " << line1.getXIntercept() << " - " << line2.getXIntercept() << std::endl;
//    cout << "y intercepts : " << line1.getYIntercept() << " - " << line2.getYIntercept() << std::endl;
//    cout << "if ^ > pi/4: x-int diff: " << abs(line1.getXIntercept() - line2.getXIntercept()) << std::endl;
//    cout << "if ^ <= pi/4: y-int diff: " << abs(line1.getYIntercept() - line2.getYIntercept()) << std::endl;
//    cout << "comb R2TLS: " << line1.combinedR2TLSandMSD(line2).x << std::endl;
//    cout << "comb MSD: " << line1.combinedR2TLSandMSD(line2).y << std::endl;

//    cout << "MAX_ANGLE_DIFF: " << MAX_ANGLE_DIFF << std::endl;
//    cout << "MIN_LINE_R2_FIT " << MIN_LINE_R2_FIT << std::endl;
//    cout << "MAX_INTERCEPT_DIFF:" << MAX_INTERCEPT_DIFF << std::endl;
//    cout << "MAX_LINE_MSD:" << MAX_LINE_MSD << std::endl;

    if(line1.getAngleBetween(line2) > MAX_ANGLE_DIFF_TO_MERGE)
        return false;

    if(abs(line1.getRho() - line2.getRho()) > MAX_DISTANCE_TO_MERGE)
        return false;


//    //check for x intercepts of nearly vertical lines
//    if(abs(line1.getAngle()) > mathGeneral::PI*0.25 && abs(line1.getXIntercept() - line2.getXIntercept()) > MAX_INTERCEPT_DIFF)
//        return false;
//    //check for y intercepts of mostly horizontal lines
//    if(abs(line1.getAngle()) <= mathGeneral::PI*0.25 && abs(line1.getYIntercept() - line2.getYIntercept()) > MAX_INTERCEPT_DIFF)
//        return false;

    Vector2<double> r2andmsd = line1.combinedR2TLSandMSD(line2);
    //ensure the line won't just be thrown out later as dirty
//    if(r2andmsd.x < MIN_LINE_R2_FIT)
//        return false;//ensure the line won't just be thrown out later as dirty
//    if(r2andmsd.y > MAX_LINE_MSD)
//        return false;

    return true; //checks passed
}
