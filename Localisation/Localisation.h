#ifndef LOCWM_H_DEFINED
#define LOCWM_H_DEFINED
#include "KF.h"

#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/GameInformation.h"
class NUSensorsData;
#include "Behaviour/TeamInformation.h"

#include "debug.h"
#include "debugverbositylocalisation.h"
#include "Tools/FileFormats/TimestampedData.h"
#include <fstream>

// Debug output level
// 0 - No messages
// 1 - Error messages
// 2 - Update messages
// 3 - All messages
// #define  DEBUG_LOCALISATION_VERBOSITY 3

class Localisation: public TimestampedData
{
	public:
        Localisation(int playerNumber = 0);
        ~Localisation();
    
        void process(NUSensorsData* data, FieldObjects* fobs, GameInformation* gameInfo, TeamInformation* teamInfo);
        //! TODO: Require robots state to be sent to enable smart model resetting.
        //! TODO: Need to add shared packets.
	
        void feedback(double*);
        double feedbackPosition[3];
        void ProcessObjects();
        bool varianceCheck(int modelID);
        int varianceCheckAll();
        void ResetAll();
        void writeToLog();
        bool doTimeUpdate(float odomForward, float odomLeft, float odomTurn);
        void WriteModelToObjects(const KF &model, FieldObjects* fobs);
        bool clipModelToField(int modelID);
        bool clipActiveModelsToField();
        int doKnownLandmarkMeasurementUpdate(StationaryObject &landmark);
        int doSharedBallUpdate(const TeamPacket::SharedBall& sharedBall);
        int doBallMeasurementUpdate(MobileObject &ball);
        int doAmbiguousLandmarkMeasurementUpdate(AmbiguousObject &ambigousObject, const vector<StationaryObject>& possibleObjects);
        int doTwoObjectUpdate(StationaryObject &landmark1, StationaryObject &landmark2);
        int getNumActiveModels();
        int getNumFreeModels();
        void ClearAllModels();
        bool CheckModelForOutlierReset(int modelID);
        int  CheckForOutlierResets();
        const KF& getBestModel() const;
        const KF& getModel(int modelNumber) const;
        int getBestModelID() const;
        void NormaliseAlphas();
        int FindNextFreeModel();
        bool MergeTwoModels(int index1, int index2);
        double MergeMetric(int index1, int index2);
        void MergeModels(int maxAfterMerge);
        void MergeModelsBelowThreshold(double MergeMetricThreshold);
        void PrintModelStatus(int modelID);

        bool IsValidObject(const Object& theObject);
	    bool amILost;               // true if we are 'lost' in this frame
	    int lostCount;              // the number of consecutive frames in which we are 'lost'
        float timeSinceFieldObjectSeen;

        // Model Reset Functions
        bool CheckGameState();
        void doInitialReset();
        void doSetReset();
        void doPenaltyReset();
        void doBallOutReset();
        void doFallenReset();
        void doReset();
        void setupModel(int modelNumber, int numModels, float x, float y, float heading);
        void setupModelSd(int modelNumber, float sdx, float sdy, float sdheading);
        void resetSdMatrix(int modelNumber);
        void swapFieldStateTeam(float& x, float& y, float& heading);

        /*!
        @brief Output streaming operation.
        @param output The output stream.
        @param p_loc The source localisation data to be streamed.
        */
        friend std::ostream& operator<< (std::ostream& output, const Localisation& p_loc);

        /*!
        @brief Input streaming operation.
        @param input The input stream.
        @param p_kf The destination localisation data to be streamed to.
        */
        friend std::istream& operator>> (std::istream& input, Localisation& p_loc);

        // Multiple Models Stuff
        static const int c_MAX_MODELS_AFTER_MERGE = 6; // Max models at the end of the frame
        static const int c_MAX_MODELS = (c_MAX_MODELS_AFTER_MERGE*8+2); // Total models
        static const int c_numOutlierTrackedObjects = FieldObjects::NUM_STAT_FIELD_OBJECTS;
        KF tempModel;
        KF models[c_MAX_MODELS];
    
        // local pointers to the public store
        NUSensorsData* m_sensor_data;
        FieldObjects* m_objects;
        GameInformation* m_game_info;
        TeamInformation* m_team_info;

	#if DEBUG_LOCALISATION_VERBOSITY > 0
        ofstream debug_file; // Logging file
        #endif // LOCWM_VERBOSITY > 0

        double m_timestamp;
        double GetTimestamp() const {return m_timestamp;};
        int currentFrameNumber;
        float modelObjectErrors[c_MAX_MODELS][c_numOutlierTrackedObjects]; // Storage of outlier history.

        // Game state memory
        bool m_previously_incapacitated;
        GameInformation::RobotState m_previous_game_state;
        
        float odomForward, odomLeft, odomTurn;
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
        static const float sdTwoObjectAngle;
	void measureLocalization(double,double,double);
};

#endif
