#ifndef SAM_H_SHANNON
#define SAM_H_SHANNON

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include "../../Tools/Math/LSFittedLine.h"
#include "../../Tools/Math/Matrix.h"
#include "../../Tools/Math/Vector3.h"


//Debug rules
#define DEBUG 0
#define DEBUG_POINTS 0
#define DEBUG_SPLIT 0
#define DEBUG_SEPARATE 0
#define DEBUG_MERGE 0
#define DEBUG_SPLIT_NOISE 0
#define DEBUG_SHOULD_SPLIT 0
#define DEBUG_CLEAR_SMALL 0

using namespace std;

class SAM
{
public:

    static vector<LinePoint*> noisePoints;
    static unsigned int noFieldLines;

    //GENERIC
    static void initDebug(ofstream& dout);
    static bool initRules(vector<unsigned int>& ints, vector<double>& doubles);
    static void initRules(double SD, unsigned int MPO, unsigned int MPTL, unsigned int MPTLF, double MEPD, double MLRF);

    //LEAST-SQUARES FITTING
    static void splitAndMergeLS(vector<LSFittedLine*>& lines, vector<LinePoint*>& points, bool clearsmall=true, bool cleardirty=true, bool noise=true);
    //CLUSTERS
    static void splitAndMergeLSClusters(vector<LSFittedLine*>& lines, vector< vector<LinePoint*> >& clusters, vector<LinePoint*> leftover, bool clearsmall=true, bool cleardirty=true, bool noise=true);
    //POINTCONVERSION
    static void convertPoint(LinePoint& point, Vector3<float>& vals);

private:
    //RULES
    //maximum field objects rules
    static unsigned int MAX_POINTS; //500
    static unsigned int MAX_LINES; //15
    //splitting rules
    static double SPLIT_DISTANCE; //1.0
    static unsigned int MIN_POINTS_OVER; //2
    //Noise splitting rules
    static unsigned int SPLIT_NOISE_ITERATIONS; //1
    //merging rules
    //#define MAX_GRAD_DIFF 1
    //#define MAX_INTERCEPT_DIFF 10
    //#define MAX_RHO_DIFF 12
    //#define MAX_PHI_DIFF 0.5
    static double MAX_END_POINT_DIFF; //5.0
    //Line keeping rules
    static unsigned int MIN_POINTS_TO_LINE; //3
    static unsigned int MIN_POINTS_TO_LINE_FINAL; //5
    static double MIN_LINE_R2_FIT; //0.90

    //DEBUGGING
    /*
    static ofstream* debug_out;

    static void debugPrint(const string& s);
    static void debugPrint(const Line& l);
    static void debugPrint(const LinePoint& p);
    static void debugPrint(const vector<LSFittedLine*>& lines);
    static void debugPrint(const vector<LinePoint*>& points);
    */

    //LEAST-SQUARES FITTING
    static void splitLS(vector<LSFittedLine*>& lines, vector<LinePoint*>& points);
    static void splitLSIterative(vector<LSFittedLine*>& lines, vector<LinePoint*>& points);
    static void splitNoiseLS(vector<LSFittedLine*>& lines);
    static void mergeLS(vector<LSFittedLine*>& lines);
    static void generateLSLine(LSFittedLine& line, vector<LinePoint*>& points);
    static void sortLinesLS(vector<LSFittedLine*>& lines);
    static bool separateLS(vector<LinePoint*>& left, vector<LinePoint*>& right, LinePoint* split_point, LSFittedLine& line);


    //GENERIC
    static void findFurthestPoint(LSFittedLine& line, int& points_over, int& furthest_point);
    static void addToNoise(LinePoint* point);
    static void clearSmallLines(vector<LSFittedLine*>& lines);
    static void clearDirtyLines(vector<LSFittedLine*>& lines);
    static bool shouldMergeLines(const LSFittedLine& line1, const LSFittedLine& line2);

};

#endif // SAM_H_SHANNON
