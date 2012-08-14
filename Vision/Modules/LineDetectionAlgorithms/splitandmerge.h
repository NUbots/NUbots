/*
 * Author: Shannon Fenn
 * Last Modified: 25/02/11
 * Description:
 *      - A static class of method implementing the split and merge line extraction
        algorithm for us on the NAO robot platform

        - Parameters are static member variable set by initRules(), rather than
        #define macros. Make sure to call this method with reasonable values
        before calling splitAndMergeLS() or splitAndMergeLSClusters()

        - the only method to call (besides initRules()) is either
         + splitAndMergeLS() - for non-clustered input
         + splitAndMergeLSClusters() - for clustered input

        - If things need to be changed the main decisions are in:
         + splitLSIterative() - decisions on whether to split, keep or throw away lines
         + findFurthestPoint() - finds the number of points distant from the line, and the furthest point
         + separateLS() - divides point set into two subsets based on the line equation and the furthest point
                uses a transform to axis defined by the line itself, and the normal from the furthest point
                to the line.
         + shouldMergeLines() - decisions on whether two lines should be merged

         - All other methods are trivial or only make simple decisions based on the parameters set
            by initRules()
 */

#ifndef SAM_H_SHANNON
#define SAM_H_SHANNON

#include <vector>
#include <math.h>
#include "Tools/Math/LSFittedLine.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Vector3.h"

using std::vector;

class SplitAndMerge
{
public:
    //GENERIC
    SplitAndMerge();
    ~SplitAndMerge();

    //LEAST-SQUARES FITTING
    vector<LSFittedLine*> run(vector<LinePoint*>& points, bool noise=true);

private:
    //RULES
    //maximum field objects rules
    unsigned int MAX_POINTS; //500
    unsigned int MAX_LINES; //15
    //splitting rules
    double SPLIT_DISTANCE; //1.0
    unsigned int MIN_POINTS_OVER; //2
    unsigned int MIN_POINTS_TO_LINE; //3
    //Noise splitting rules
    unsigned int SPLIT_NOISE_ITERATIONS; //1
    //merging rules
    double MAX_END_POINT_DIFF; //5.0
    double MAX_ANGLE_DIFF;
    double MAX_INTERCEPT_DIFF;
    //Line keeping rules
    unsigned int MIN_POINTS_TO_LINE_FINAL; //5
    double MIN_LINE_R2_FIT; //0.90
    double MAX_LINE_MSD; //50 set at constructor
    //clearing rules
    bool CLEAR_SMALL;
    bool CLEAR_DIRTY;

    vector<LinePoint*> noisePoints;

    //LEAST-SQUARES FITTING
    void split(vector<LSFittedLine*>& lines, vector<LinePoint*>& points);
    void splitIterative(vector<LSFittedLine*>& lines, vector<LinePoint*>& points);
    void splitNoise(vector<LSFittedLine*>& lines);
    void merge(vector<LSFittedLine*>& lines);
    void generateLine(LSFittedLine& line, vector<LinePoint*>& points);
    bool separate(vector<LinePoint*>& left, vector<LinePoint*>& right, LinePoint* split_point, LSFittedLine& line);
    //static void sortLinesLS(vector<LSFittedLine*>& lines);


    //GENERIC
    void findPointsOver(LSFittedLine& line, unsigned int &points_over, int& furthest_point);
    void addToNoise(LinePoint* point);
    void addToNoise(const vector<LinePoint*>& points);
    void clearSmallLines(vector<LSFittedLine*>& lines);
    void clearDirtyLines(vector<LSFittedLine*>& lines);
    bool shouldMergeLines(const LSFittedLine& line1, const LSFittedLine& line2);

};

#endif // SAM_H_SHANNON
