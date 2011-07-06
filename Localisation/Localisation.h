#ifndef LOCWM_H_DEFINED
#define LOCWM_H_DEFINED
#include "KF.h"

#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/GameInformation/GameInformation.h"
class NUSensorsData;
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "debug.h"
#include "debugverbositylocalisation.h"
#include "Tools/FileFormats/TimestampedData.h"
#include <fstream>
#include <sstream>

// Debug output level
// 0 - No messages
// 1 - Error messages
// 2 - Update messages
// 3 - All messages
// #define  DEBUG_LOCALISATION_VERBOSITY 3

#define LOC_SUMMARY 0

class Localisation: public TimestampedData
{
	public:
        Localisation(int playerNumber = 0);
        Localisation(const Localisation& source);
        ~Localisation();
    
        void process(NUSensorsData* data, FieldObjects* fobs, const GameInformation* gameInfo, const TeamInformation* teamInfo);
        //! TODO: Require robots state to be sent to enable smart model resetting.
        //! TODO: Need to add shared packets.
	
        void feedback(double*);
        double feedbackPosition[3];
        void ProcessObjects(FieldObjects* fobs, const vector<TeamPacket::SharedBall>& sharedballs, float time_increment);
        bool varianceCheck(int modelID, FieldObjects* fobs);
        int varianceCheckAll(FieldObjects* fobs);
        void ResetAll();
        void writeToLog();
        bool doTimeUpdate(float odomForward, float odomLeft, float odomTurn, double time_increment);
        void WriteModelToObjects(const KF &model, FieldObjects* fobs);
        bool clipModelToField(int modelID);
        bool clipActiveModelsToField();

        int doMultipleKnownLandmarkObservationUpdate(std::vector<StationaryObject*>& landmarks);
        int doKnownLandmarkMeasurementUpdate(StationaryObject &landmark);
        int doSharedBallUpdate(const TeamPacket::SharedBall& sharedBall);
        int doBallMeasurementUpdate(MobileObject &ball);
        int doAmbiguousLandmarkMeasurementUpdate(AmbiguousObject &ambigousObject, const vector<StationaryObject>& possibleObjects);
        int doAmbiguousLandmarkMeasurementUpdateDiscard(AmbiguousObject &ambigousObject, const vector<StationaryObject>& possibleObjects);
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
        std::string ModelStatusSummary();

        void removeAmbiguousGoalPairs(std::vector<AmbiguousObject>& ambiguousobjects, bool yellow_seen, bool blue_seen);

		void resetPlayingStateModels();

        bool IsValidObject(const Object& theObject);
        bool amILost;                       // true if we are 'lost' in this frame
        int lostCount;                      // the number of consecutive frames in which we are 'lost'
        float timeSinceFieldObjectSeen;     // the time since a useful field object has been seen

        // Model Reset Functions
        void initSingleModel(float x, float y, float theta);
        bool CheckGameState(bool currently_incapacitated, const GameInformation *game_info);
        void doInitialReset(GameInformation::TeamColour team_colour);
        void doSetReset(GameInformation::TeamColour team_colour, int player_number, bool have_kickoff);
        void doPenaltyReset();
        void doBallOutReset();
        void doFallenReset();
        void doReset();
        void setupModel(int modelNumber, int numModels, float x, float y, float heading);
        void setupModelSd(int modelNumber, float sdx, float sdy, float sdheading);
        void resetSdMatrix(int modelNumber);
        void swapFieldStateTeam(float& x, float& y, float& heading);

        std::vector<TeamPacket::SharedBall> FindNewSharedBalls(const std::vector<TeamPacket::SharedBall>& allSharedBalls);

        std::string frameLog() const
        {
            return m_frame_log.str();
        }

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

        Localisation& operator= (const Localisation & source);


        // Multiple Models Stuff
        static const int c_MAX_MODELS_AFTER_MERGE = 6; // Max models at the end of the frame
        static const int c_MAX_MODELS = (c_MAX_MODELS_AFTER_MERGE*8+2); // Total models
        static const int c_numOutlierTrackedObjects = FieldObjects::NUM_STAT_FIELD_OBJECTS;
        KF m_tempModel;
        KF m_models[c_MAX_MODELS];

	#if DEBUG_LOCALISATION_VERBOSITY > 0
        ofstream debug_file; // Logging file
        #endif // LOCWM_VERBOSITY > 0

        double m_timestamp;
        double GetTimestamp() const {return m_timestamp;};
        int m_currentFrameNumber;
        float m_modelObjectErrors[c_MAX_MODELS][c_numOutlierTrackedObjects]; // Storage of outlier history.

        std::vector<TeamPacket::SharedBall> m_prevSharedBalls;

        // Game state memory
        bool m_previously_incapacitated;
        GameInformation::RobotState m_previous_game_state;
        std::stringstream m_frame_log;

        std::vector<float> m_gps;
        float m_compass;
        bool m_hasGps;
        
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
};

#endif
