#ifndef LOCWM_H_DEFINED
#define LOCWM_H_DEFINED
#include "KF.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include <fstream>

#define MULTIPLE_MODELS_ON 1
#define AMBIGUOUS_CORNERS_ON 0
#define SHARED_BALL_ON 1

// Debug output level
// 0 - No messages
// 1 - Error messages
// 2 - Update messages
// 3 - All messages
#define LOCWM_VERBOSITY 3

class LocWM
{
	public:
        LocWM();
        ~LocWM();
        //! TODO: Require robots state to be sent to enable smart model resetting.
        //! TODO: Need to add shared packets.
        void ProcessObjects(int frameNumber, FieldObjects* ourfieldObjects, void* mostRecentPackets);
        void CheckGameState();
        bool varianceCheck(int modelID);
        int varianceCheckAll();
        void ResetAll();
        void writeToLog();
        bool doTimeUpdate(float odomForward, float odomLeft, float odomTurn);
        bool clipModelToField(int modelID);
        bool clipActiveModelsToField();
        int doKnownLandmarkMeasurementUpdate(StationaryObject &landmark);
        //int doSharedBallUpdate(WirelessFieldObj &sharedBall);
        int doBallMeasurementUpdate(MobileObject &ball);
        int doAmbiguousLandmarkMeasurementUpdate(AmbiguousObject &ambigousObject, const std::vector<StationaryObject>& possibleObjects);
        int getNumActiveModels();
        int getNumFreeModels();
        bool CheckModelForOutlierReset(int modelID);
        int  CheckForOutlierResets();
        KF &getBestModel();
        int getBestModelID();
        void NormaliseAlphas();
        int FindNextFreeModel();
        bool MergeTwoModels(int index1, int index2);
        double MergeMetric(int index1, int index2);
        void MergeModels(int maxAfterMerge);
	    void MergeModelsBelowThreshold(double MergeMetricThreshold);
        void PrintModelStatus(int modelID);


        // Model Reset Functions
        void doPenaltyReset();
        void doPlayerReset();
        void resetSdMatrix(int modelNumber);

        // Multiple Models Stuff
        static const int c_MAX_MODELS_AFTER_MERGE = 4; // Max models at the end of the frame
        static const int c_MAX_MODELS = (c_MAX_MODELS_AFTER_MERGE*8+2); // Total models
        static const int c_numOutlierTrackedObjects = FieldObjects::NUM_STAT_FIELD_OBJECTS;
        KF tempModel;
        KF models[c_MAX_MODELS];
        FieldObjects *objects;

        #if LOCWM_VERBOSITY > 0
        fstream debug_file; // Logging file
        #endif // LOCWM_VERBOSITY > 0

        int currentFrameNumber;
        float modelObjectErrors[c_MAX_MODELS][c_numOutlierTrackedObjects]; // Storage of outlier history.

        // Game state memory
        bool wasPreviouslyPenalised;
        int previousGameState;

        // Tuning Constants -- Values assigned in LocWM.cpp
        static const float c_LargeAngleSD;
        static const float c_OBJECT_ERROR_THRESHOLD;
        static const float c_OBJECT_ERROR_DECAY;
        static const float c_RESET_SUM_THRESHOLD;
        static const int c_RESET_NUM_THRESHOLD;

        // Object distance measurement error weightings (Constant) -- Values assigned in LocWM.cpp
        static const float R_obj_theta;
        static const float R_obj_range_offset;
        static const float R_obj_range_relative;
        static const float centreCircleBearingError;
};

#endif
