#include "SAM.h"
//#include <QDebug>

using std::vector;

vector<LinePoint*> SAM::noisePoints;
unsigned int SAM::noFieldLines;
unsigned int SAM::MAX_LINES, SAM::MAX_POINTS, SAM::MIN_POINTS_OVER, SAM::MIN_POINTS_TO_LINE, SAM::MIN_POINTS_TO_LINE_FINAL, SAM::SPLIT_NOISE_ITERATIONS;
double SAM::MAX_END_POINT_DIFF, SAM::MIN_LINE_R2_FIT, SAM::SPLIT_DISTANCE;
//ofstream* SAM::debug_out;
/*
std::vector<LinePoint*> SAM::linePoints;
std::vector<LSFittedLine*> SAM::fieldLines;
std::vector<Line*> SAM::EfieldLines;
*/

//DEBUG
/*
void SAM::initDebug(ofstream& dout) {
    debug_out = &dout;
    if(*debug_out) {
        *debug_out << "Debug Options: \n";
        *debug_out << "DEBUG " << DEBUG << "\n";
        *debug_out << "DEBUG_POINTS " << DEBUG_POINTS << "\n";
        *debug_out << "DEBUG_SPLIT " << DEBUG_SPLIT << "\n";
        *debug_out << "DEBUG_SEPARATE " << DEBUG_SEPARATE << "\n";
        *debug_out << "DEBUG_MERGE " << DEBUG_MERGE << "\n";
        *debug_out << "DEBUG_SPLIT_NOISE " << DEBUG_SPLIT_NOISE << "\n";
        *debug_out << "DEBUG_SHOULD_SPLIT " << DEBUG_SHOULD_SPLIT << "\n";
        *debug_out << "DEBUG_CLEAR_SMALL " << DEBUG_CLEAR_SMALL << "\n";
    }
}


void SAM::debugPrint(const string& s) {
    if(*debug_out) {
        *debug_out << s;
        *debug_out << "\n";
    }
}


void SAM::debugPrint(const Line& l) {
    if(*debug_out) {
        *debug_out << l.getA();
        *debug_out << "x + ";
        *debug_out << l.getB();
        *debug_out << "y = ";
        *debug_out << l.getC();
        *debug_out << "\n";
    }
}


void SAM::debugPrint(const LinePoint& p) {
    if(*debug_out) {
        *debug_out << "  ";
        *debug_out << "(";
        *debug_out << p.x;
        *debug_out << ",";
        *debug_out << p.y;
        *debug_out << ")\n";
    }
}

void SAM::debugPrint(const vector<LSFittedLine *> &lines) {
    if(*debug_out) {
        for(unsigned int i=0; i<lines.size(); i++) {
            *debug_out << "  ";
            debugPrint(*lines[i]);
        }
    }
}

void SAM::debugPrint(const vector<LinePoint *> &points) {
    if(DEBUG_POINTS) {
        if(*debug_out) {
            for(unsigned int i=0; i<points.size(); i++) {
                debugPrint(*points[i]);
            }
        }
    }
}

*/
bool SAM::initRules(vector<unsigned int>& ints, vector<double>& doubles) {
    if(ints.size() == 3 && doubles.size() == 3) {
        initRules(doubles[0], ints[1], ints[1], ints[2], doubles[1], doubles[2]);
        return true;
    }
    else {
        return false;
    }
}

void SAM::initRules(double SD, unsigned int MPO, unsigned int MPTL, unsigned int MPTLF, double MEPD, double MLRF) {
    SPLIT_DISTANCE = SD;
    MIN_POINTS_OVER = MPO;
    MAX_END_POINT_DIFF = MEPD;
    MIN_POINTS_TO_LINE = MPTL;
    MIN_POINTS_TO_LINE_FINAL = MPTLF;
    MIN_LINE_R2_FIT = MLRF;
    MAX_POINTS = 500;
    MAX_LINES = 15;
    SPLIT_NOISE_ITERATIONS = 1;
}

//CLUSTERS
void SAM::splitAndMergeLSClusters(vector<LSFittedLine*>& lines, vector< vector<LinePoint*> >& clusters, vector<LinePoint*> leftover, bool clearsmall, bool cleardirty, bool noise) {

//qDebug() << "Iterative split";
    noFieldLines = 0;
    noisePoints.clear();

    //For each cluster..
//qDebug() << "Cluster: ";
    for(unsigned int i=0; i<clusters.size(); i++) {
        //perform split - splitLS() checks for appropriate size, so that need not be done here
        //qDebug() << i;
        splitLSIterative(lines, clusters[i]);
    }
//qDebug() << "leftover";
    //Then split leftover
    splitLSIterative(lines, leftover);
//qDebug() << "noise";
    //Then noise
    if(noise) {
        for(unsigned int i=0; i<SPLIT_NOISE_ITERATIONS; i++) {
            splitNoiseLS(lines);
        }
    }
    //Do Centre Circle fitting before merge


//qDebug() << "merge";
    //Then Merge
    mergeLS(lines);
//qDebug() << "clearsmall";
    //Then clear unwanted lines
    if(clearsmall) {
        clearSmallLines(lines);
    }
//qDebug() << "cleardirty";
    if(cleardirty) {
        clearDirtyLines(lines);
    }


    noisePoints.clear();
//qDebug() << "finished";
}


//LEAST-SQUARE FITTING

void SAM::splitAndMergeLS(vector<LSFittedLine*>& lines, vector<LinePoint*>& points, bool clearsmall, bool cleardirty, bool noise) {
    /*
     *
     */
    noFieldLines = 0;
    noisePoints.clear();

    splitLS(lines, points);

    if(noise) {
        for(unsigned int i=0; i<SPLIT_NOISE_ITERATIONS; i++) {
            splitNoiseLS(lines);
        }
    }

    mergeLS(lines);

    if(clearsmall)
        clearSmallLines(lines);

    if(cleardirty)
        clearDirtyLines(lines);

    noisePoints.clear();
}



void SAM::splitLS(vector<LSFittedLine*>& lines, vector<LinePoint*>& points) {
    //Assumes:
    //	- constant detirmined for limit - MIN_POINTS_OVER
    //	- constant for min splitting distance - SAM_THRESHOLD

    //Boundary Conds
    if(noFieldLines >= MAX_LINES) {
        return;
    }
    if(points.size() < MIN_POINTS_TO_LINE) {
        //add points to noise
        for(unsigned int i=0; i<points.size(); i++)
            addToNoise(points[i]);
        return;
    }

    //temp variables
    double distance = 0.0; 	//holder for calculated LinePointDistance
    int points_over = 0; //how many points are further from the line than SAM_THRESHOLD
    int greatest_point = 0; //which point in vector is the furthest
    double greatest_distance = 0.0; //saves recalculation of greatest point distance
    unsigned int current_point;

    //LinePoint* temppoint;


    //generate new LSFittedLine
    LSFittedLine* line = new LSFittedLine();

    generateLSLine(*line, points);

    /*
    //check points for perp distance over threshold
    for(current_point = 0; current_point < points.size(); current_point++) {
        distance = line->getLinePointDistance(*(points[current_point]));
        if(distance > SPLIT_DISTANCE) {
            //potential splitting point
            points_over++; //increment points_over counter
            //check if greatest point
            if(distance > greatest_distance) {
                    //new furthest point found
                    greatest_distance = distance;
                    greatest_point = current_point;
            }
        }
    }
    */
    //check for points over threshold
    findFurthestPoint(*line, points_over, greatest_point);

    //if num points over threshold > limit -> split at greatest distance point.
    if(points_over >= MIN_POINTS_OVER) {
        //there are enough points distant to justify a split
        vector<LinePoint*> left;
        vector<LinePoint*> right;
        if(separateLS(left, right, points[greatest_point], *line)) {
            if(left.size() >= MIN_POINTS_TO_LINE) {
                splitLS(lines, left);
            }
            else {
                //add left to noise Points
                for(unsigned int i=0; i<left.size(); i++)
                    addToNoise(left[i]);
            }
            if(right.size() >= MIN_POINTS_TO_LINE) {
                splitLS(lines, right);
            }
            else {
                //add right to noise Points
                for(unsigned int i=0; i<right.size(); i++)
                    addToNoise(right[i]);
            }
        }
        else {
            //remove furthest point and retry
            vector<LinePoint*> newlist = points;
            addToNoise(newlist[greatest_point]);
            for(unsigned int i=greatest_point; i<newlist.size()-1; i++) {
                newlist[i] = newlist[i+1];
            }
            newlist[newlist.size()-1] = 0;
            newlist.pop_back();
            splitLS(lines, newlist);
        }
    }
    else if(points_over > 0) {
        //not enough points over to split so remove point as noisy, and regen line
        if(points.size() > MIN_POINTS_TO_LINE_FINAL) {
            //i.e. removal of a point will still leave enough to form a reasonable line
            vector<LinePoint*> newlist = points;
            addToNoise(newlist[greatest_point]);
            for(unsigned int i=greatest_point; i<newlist.size()-1; i++) {
                newlist[i] = newlist[i+1];
            }
            newlist[newlist.size()-1] = 0;
            newlist.pop_back();
            delete line; //clear memory of old line
            line = new LSFittedLine();
            generateLSLine(*line, newlist);
            lines.push_back(line);
        }
        else {
            //NOT SURE ??
            //Add points to noise and delete line
            //lines.push_back(line);
            for(unsigned int i=0; i<points.size(); i++)
                addToNoise(points[i]);
            line->getPoints().clear();
            delete line;

        }
    }
    else {
        //no points over, just push line
        lines.push_back(line);
    }
}

void SAM::splitLSIterative(vector<LSFittedLine*>& lines, vector<LinePoint*>& points) {
//qDebug() << "points size: " << points.size();

    //Boundary Conds
    if(noFieldLines >= MAX_LINES) {
        return;
    }
    if(points.size() < MIN_POINTS_TO_LINE) {
        //add points to noise
        for(unsigned int i=0; i<points.size(); i++)
            addToNoise(points[i]);
        return;
    }

    //Locals
    vector<LSFittedLine*> stack;
    stack.clear();
    int furthest_point, points_over;
    vector<LinePoint*> left, right;

    LSFittedLine* tempLine = new LSFittedLine();
    //generate first line
    generateLSLine(*tempLine, points);
    //push line onto stack
    stack.push_back(tempLine);

    //Begin iteration
    while(!stack.empty() && noFieldLines + stack.size() < MAX_LINES) {
        //Pop the top line off and split it if warranted
        //if not, slap it on the end of lines and go again
        //until stack is empty or maximum lines reached
//qDebug() << stack.size();
        //Clear left and right
        left.clear();
        right.clear();

        //pop top line
        tempLine = stack.back();
        stack.pop_back();

        //check for points over threshold
        findFurthestPoint(*tempLine, points_over, furthest_point);

        //Options
        if(points_over >= MIN_POINTS_OVER) {
            //See if separation is an option
            if(separateLS(left, right, tempLine->getPoints()[furthest_point], *tempLine)) {
                //qDebug() << "separating worked";
                //clear old line
                tempLine->getPoints().clear();
                delete tempLine;
                //check if left is big enough
                if(left.size() >= MIN_POINTS_TO_LINE) {
                    //generate line and push it to stack
                    tempLine = new LSFittedLine();
                    generateLSLine(*tempLine, left);
                    stack.push_back(tempLine);
                }
                else {
                    //throw left points to noise
                    for(unsigned int i=0; i<left.size(); i++)
                        addToNoise(left[i]);
                    left.clear();
                }
                //check if right is big enough
                if(right.size() >= MIN_POINTS_TO_LINE) {
                    //generate line and push it to stack
                    tempLine = new LSFittedLine();
                    generateLSLine(*tempLine, right);
                    stack.push_back(tempLine);
                }
                else {
                    //throw right points to noise
                    for(unsigned int i=0; i<right.size(); i++)
                        addToNoise(right[i]);
                    right.clear();
                }
            } //if(separate())
            else {
                //Separation didn't work
                //remove furthest point and push new line
                vector<LinePoint*> newlist = tempLine->getPoints();
                //remove noisy point and update points list
                addToNoise(newlist[furthest_point]);
                newlist[furthest_point] = newlist[newlist.size() - 1];
                newlist[newlist.size()-1] = 0;
                newlist.pop_back();
                //clear old line
                tempLine->getPoints().clear();
                delete tempLine;
                tempLine = new LSFittedLine();
                generateLSLine(*tempLine, newlist); //make new line
                stack.push_back(tempLine); //push it back
            }
        } //if(points_over >= MIN_POINTS_OVER)
        else if(points_over > 0){
            //not enough points over to split - but there are points over
            if(tempLine->getPoints().size() > MIN_POINTS_TO_LINE_FINAL) {
                //i.e. removal of a point will still leave enough to form a reasonable line
                //remove noisy point and regen line
                vector<LinePoint*> newlist = tempLine->getPoints();
                //remove noisy point and update points list
                addToNoise(newlist[furthest_point]);
                newlist[furthest_point] = newlist[newlist.size() - 1];
                newlist[newlist.size()-1] = 0;
                newlist.pop_back();
                tempLine->getPoints().clear();
                delete tempLine; //clear memory of old line
                tempLine = new LSFittedLine();
                generateLSLine(*tempLine, newlist); //make new line
                //qDebug() << "pushing line";
                lines.push_back(tempLine); //push it back to finals
                noFieldLines++;
            }
            else {
                //Add points to noise and delete line
                for(unsigned int i=0; i<tempLine->getPoints().size(); i++)
                    addToNoise(tempLine->getPoints()[i]);
                tempLine->getPoints().clear();
                delete tempLine;
            }
        } //elseif(points_over > 0)
        else {
            //no points over, just push line to finals
            //qDebug() << "pushing line";
            lines.push_back(tempLine);
            noFieldLines++;
        }
    } // While(!stack.empty() && noFieldLines + stack.size() <= MAX_LINES)


//qDebug() << stack.size();
    //If MAX_LINES reached, but stack is not empty push stack lines to finals
    if(noFieldLines + stack.size() == MAX_LINES) {
        //qDebug() << "Maximum Lines reached in split";
    }
    while(!stack.empty()) {
        lines.push_back(stack.back());
        stack.pop_back();
    }
}

void SAM::findFurthestPoint(LSFittedLine& line, int& points_over, int& furthest_point) {

    //temp variables
    double distance = 0.0; 	//holder for calculated LinePointDistance
    double greatest_distance = 0.0; //saves recalculation of greatest point distance
    unsigned int current_point = 0;
    points_over = 0;
    furthest_point = -1;
    vector<LinePoint*> points = line.getPoints();

    //check points for perp distance over threshold
    for(current_point = 0; current_point < points.size(); current_point++) {
        distance = line.getLinePointDistance(*(points[current_point]));
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
}

void SAM::splitNoiseLS(vector<LSFittedLine*>& lines) {

    if(noisePoints.size() >= MIN_POINTS_TO_LINE_FINAL) {
        vector<LinePoint*> noiseCopy;

        noiseCopy = noisePoints;
        noisePoints.clear();
        splitLSIterative(lines, noiseCopy);
    }
}


bool SAM::separateLS(vector<LinePoint*>& left, vector<LinePoint*>& right, LinePoint* split_point, LSFittedLine& line) {
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
        double A = line.getA();
        double B = line.getB();
        double C = line.getC();
        double x1 = split_point->x;
        double y1 = split_point->y;
        vector<LinePoint*> points = line.getPoints();
        //vector<LinePoint*> points = *points;

        left.push_back(split_point);
        right.push_back(split_point);
/*****DEBUGGING OUTPUT******/
        //printf("\nSplitting on: (%.2f,%.2f)\n",x1,y1);
/*****DEBUGGING OUTPUT******/
        if(A==0.0) {
            //horizontal line - no rotation
            //x' = x - x0
            for(unsigned int pointcounter = 0; pointcounter < points.size(); pointcounter++) {
                //check all points, calculate translated x coord
                //and place in appropriate vector
                if(!(points[pointcounter] == split_point)) {
                    if(points[pointcounter]->x < x1) {
                        //point is to the left
                        left.push_back(points[pointcounter]);
                    }
                    else {
                        right.push_back(points[pointcounter]);
                    }
                }
            }
        }
        else if(B==0.0) {
            //vertical line - 90 degree rotation
            //x' = y - y0
            for(unsigned int pointcounter = 0; pointcounter < points.size(); pointcounter++) {
                //check all points, calculate translated x coord
                //and place in appropriate vector
                if(!(points[pointcounter] == split_point)) {
                    if(points[pointcounter]->y < y1) {
                        //point is to the left
                        left.push_back(points[pointcounter]);
                    }
                    else {
                        right.push_back(points[pointcounter]);
                    }
                }
            }
        }
        else {
         // STILL TO DO: FIX MATRIX CALCS

            //sloped line
            //x' = (x - x0)cosa + (y - y0)sina
            //a = atan(-A/B)
            //(x0,y0) from [ a, b; b, -a][x; y] = [c; bx1 - ay1]
            double alpha = atan(-A/B);
            double xtrans;
            // Mx = k  ->  x = M^-1k
            Matrix* M = new Matrix(2,2);
            Matrix Minv;
            Matrix* k = new Matrix(2,1);
            Matrix* X = new Matrix(2,1);
            double *r0 = (*M)[0]; //row 0
            double *r1 = (*M)[1]; //row 1
            //M = |A   B|
            //    |B  -A|
            *r0 = A;
            *(r0+1) = B;
            *r1 = B;
            *(r1+1) = -A;

            // k = |    c    |
            //     | bx1-ay1 |
            r0 = (*k)[0];
            *r0 = C;
            *(r0+1) = (B*x1) - (A*y1);

            //invert M
            Minv = Invert22(*M);

            *X = Minv*(*k);

            for(unsigned int pointcounter = 0; pointcounter <points.size(); pointcounter++) {
                //check all points, calculate translated x coord
                //and place in appropriate vector
                if(!(points[pointcounter] == split_point)) {
                    double tempx = points[pointcounter]->x;
                    double tempy = points[pointcounter]->y;
                    xtrans = (tempx - *(*X)[0])*cos(alpha) + (tempy - *(*X)[1])*sin(alpha);
                    if(xtrans < 0) {
                        //point is to the left
                        left.push_back(points[pointcounter]);
                    }
                    else {
                        right.push_back(points[pointcounter]);
                    }
                }
            }
        }
        //if either left or right contains entire point set then there will be an
        //infinite loop
        //qDebug() << "Old line size: " << line.getPoints().size() << " Left size: " << left.size() << " Right size: " << right.size();
        if(left.size() == points.size() || right.size() == points.size())
            return false;
        return true;

}


void SAM::mergeLS(vector<LSFittedLine*>& lines) {
    //O(l^2)  -  l=number of lines (max 15)


    vector<LSFittedLine*> finals; //this vector contains lines that did not need merging

    unsigned int i; //counter
    LSFittedLine* current; //pointer to current line


    while(!lines.empty()) {
        //get next line
        current = lines.back();
        lines.pop_back();
        //go through all lines and find any that should be merged - merge them
        for(i=0; i<lines.size(); i++) {
            if(shouldMergeLines(*current, *lines[i])) {
                //join the lines
                current->joinLine(*(lines[i]));
                //remove the considered line and update line list
                delete lines[i];
                if(i<lines.size())
                    lines[i] = lines.back();
                lines.pop_back();

                //need to check against newly inserted line
                i--;
            }
        }
        //Now current should have been merged with any valid lines
        //push current to finals
        finals.push_back(current);
    }
    while(!finals.empty()) {
        lines.push_back(finals.back());
        finals.pop_back();
    }
}



void SAM::generateLSLine(LSFittedLine& line, vector<LinePoint*>& points) {
    line.clearPoints();
    for(unsigned int i = 0; i < points.size(); i++) {
        line.addPoint(*points[i]);
    }
}




//GENERIC

void SAM::addToNoise(LinePoint* point) {
    //NOT EFFICIENT
    //O(n) for every insertion
    bool add = true;
    for(unsigned int i=0; i<noisePoints.size(); i++) {
        if(*noisePoints[i] == *point)
            add = false;
    }
    if(add)
        noisePoints.push_back(point);
}


void SAM::clearSmallLines(vector<LSFittedLine*>& lines) {
    vector<LSFittedLine*> stack;
    LSFittedLine* l1;
    while(!lines.empty()) {
        l1 = lines.back();
        if(l1->getPoints().size() >= MIN_POINTS_TO_LINE_FINAL) {
            //keep line
            stack.push_back(l1);
        }
        else {
            delete l1;
        }
        //else throw line away
        lines.pop_back();
    }
    while(!stack.empty()) {
        lines.push_back(stack.back());
        stack.pop_back();
    }
}



void SAM::clearDirtyLines(vector<LSFittedLine*>& lines) {
    vector<LSFittedLine*> stack;
    LSFittedLine* l1;
    while(!lines.empty()) {
        l1 = lines.back();
        if(l1->getr2tls() >= MIN_LINE_R2_FIT) {
            //keep line
            stack.push_back(l1);
        }
        else {
            delete l1;
        }
        //else throw line away
        lines.pop_back();
    }
    while(!stack.empty()) {
        lines.push_back(stack.back());
        stack.pop_back();
    }
}

bool SAM::shouldMergeLines(const LSFittedLine& line1, const LSFittedLine& line2){
    //END POINT METHOD
    bool endPointsGood;

    //find outermost end points x vals
    double farLeftX, farRightX;
    if(line1.leftPoint.x <= line2.leftPoint.x)
        farLeftX = line1.leftPoint.x;
    else
        farLeftX = line2.leftPoint.x;
    if(line1.rightPoint.x > line2.rightPoint.x)
        farRightX = line1.rightPoint.x;
    else
        farRightX = line2.rightPoint.x;
    //calc y vals for outer points for both lines
    double leftY1, leftY2, rightY1, rightY2;
    //line1
    leftY1 = line1.findYFromX(farLeftX);
    rightY1 = line1.findYFromX(farRightX);
    //line2
    leftY2 = line2.findYFromX(farLeftX);
    rightY2 = line2.findYFromX(farRightX);
    //differences check:
    endPointsGood = (fabs(leftY1-leftY2) <= MAX_END_POINT_DIFF) && (fabs(rightY1-rightY2) <= MAX_END_POINT_DIFF);


    //R2TLS
    Vector2<double> results = line1.combinedR2TLSandMSD(line2);
    bool r2good = (results.x >= line1.getr2tls()) && (results.x >= line2.getr2tls());

    return endPointsGood && r2good;

    //RHO/PHI - Not working properly
    /*
    if(DEBUG_MERGE) {
        debugPrint("\ncomparing:");
        debugPrint(line1);
        *debug_out << "   Rho: " << line1.getRho() << " Phi: " << line1.getPhi() << "\n";
        debugPrint(line2);
        *debug_out << "   Rho: " << line2.getRho() << " Phi: " << line2.getPhi() << "\n";
        debugPrint("   differences:");
        *debug_out << "      Rho: " << fabs(line1.getRho() - line2.getRho()) <<
                " Phi: " << fabs(line1.getPhi() - line2.getPhi()) << "\n";
    }


    //checks if lines should be merged
    if(fabs(line1.getRho()-line2.getRho()) <= MAX_RHO_DIFF )
        if(fabs(line1.getPhi() - line2.getPhi()) <= MAX_PHI_DIFF)
            return true;
    return false;
    */
    //OLD
    /*
    if(line1.isVertical() && line2.isVertical()) {
        if(fabs(line1.getXIntercept() - line2.getXIntercept()) <= MAX_INTERCEPT_DIFF) {
            //MERGE VERTICAL LINES
            if(DEBUG_MERGE)
                debugPrint("Will Merge");
            return true;
        }
    }
    else if(!line1.isVertical() && !line2.isVertical()) {
       //both sloped or horiz
        if(fabs(line1.getGradient() - line2.getGradient()) <= MAX_GRAD_DIFF) {
            //similar gradients
            if(fabs(line1.getYIntercept() - line2.getYIntercept()) <= MAX_INTERCEPT_DIFF) {
                //similar intercepts, same line
                if(DEBUG_MERGE)
                    debugPrint("Will Merge");
                return true;
            }
        }
    }
    //conditions for merging not satisfied
    return false;
    */
}
