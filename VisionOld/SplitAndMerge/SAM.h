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
//#include <iostream>
//#include <fstream>
#include "Tools/Math/LSFittedLine.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Vector3.h"

//Debug rules
#define DEBUG 0
#define DEBUG_POINTS 0
#define DEBUG_SPLIT 0
#define DEBUG_SEPARATE 0
#define DEBUG_MERGE 0
#define DEBUG_SPLIT_NOISE 0
#define DEBUG_SHOULD_SPLIT 0
#define DEBUG_CLEAR_SMALL 0

class Vision;
class LineDetection;



class SAM
{
public:

    static std::vector<LinePoint*> noisePoints;
    static std::vector<LinePoint*> noisePoints1;
    static std::vector<LinePoint*> noisePoints2;
    static unsigned int noFieldLines;

    //GENERIC
    //static void initDebug(ofstream& dout);
    static void initRules(double SD, unsigned int MPO, unsigned int MPTL, unsigned int MPTLF, double MEPD, double MLRF);

    //LEAST-SQUARES FITTING
    static void splitAndMergeLS(std::vector<LSFittedLine*>& lines, std::vector<LinePoint*>& points, bool clearsmall=true, bool cleardirty=true, bool noise=true);
    //CLUSTERS
    static void splitAndMergeLSClusters(std::vector<LSFittedLine*>& lines, std::vector< std::vector<LinePoint*> >& clusters, std::vector<LinePoint*> leftover, Vision* vision, LineDetection* linedetector, bool clearsmall=true, bool cleardirty=true, bool noise=true);

private:
    //RULES
    //maximum field objects rules
    static unsigned int MAX_POINTS; //500
    static unsigned int MAX_LINES; //15
    //splitting rules
    static double SPLIT_DISTANCE; //1.0
    static unsigned int MIN_POINTS_OVER; //2
    static unsigned int MIN_POINTS_TO_LINE; //3
    //Noise splitting rules
    static unsigned int SPLIT_NOISE_ITERATIONS; //1
    //merging rules
    static double MAX_END_POINT_DIFF; //5.0
    //Line keeping rules
    static unsigned int MIN_POINTS_TO_LINE_FINAL; //5
    static double MIN_LINE_R2_FIT; //0.90
    static double MAX_LINE_MSD; //50 set at constructor
    //DEBUGGING
    /*
    static ofstream* debug_out;

    static void debugPrint(const std::string& s);
    static void debugPrint(const Line& l);
    static void debugPrint(const LinePoint& p);
    static void debugPrint(const std::vector<LSFittedLine*>& lines);
    static void debugPrint(const std::vector<LinePoint*>& points);
    */

    //LEAST-SQUARES FITTING
    static void splitLS(std::vector<LSFittedLine*>& lines, std::vector<LinePoint*>& points);
    static void splitLSIterative(std::vector<LSFittedLine*>& lines, std::vector<LinePoint*>& points);
    static void splitNoiseLS(std::vector<LSFittedLine*>& lines);
    static void splitNoiseLS12(std::vector<LSFittedLine*>& lines);
    static void mergeLS(std::vector<LSFittedLine*>& lines, LineDetection* lineDetector, Vision* vision);
    static void generateLSLine(LSFittedLine& line, std::vector<LinePoint*>& points);
    static bool separateLS(std::vector<LinePoint*>& left, std::vector<LinePoint*>& right, std::vector<LinePoint*>& below, std::vector<LinePoint*>& above, std::vector<LinePoint*>& centre, LinePoint* split_point, LSFittedLine& line, bool& useTriple);
    //static void sortLinesLS(std::vector<LSFittedLine*>& lines);


    //GENERIC
    static void findFurthestPoint(LSFittedLine& line, int& points_over, int& furthest_point);
    static void addToNoise(LinePoint* point);
    static void addToNoise1(LinePoint* point);
    static void addToNoise2(LinePoint* point);
    static void clearSmallLines(std::vector<LSFittedLine*>& lines);
    static void clearDirtyLines(std::vector<LSFittedLine*>& lines);
    static bool shouldMergeLines(const LSFittedLine& line1, const LSFittedLine& line2, LineDetection* lineDetector, Vision* vision);
    static bool convertLinesEndPoints(std::vector<LSFittedLine*>& lines, Vision* vision, LineDetection* linedetector);

};

#endif // SAM_H_SHANNON
