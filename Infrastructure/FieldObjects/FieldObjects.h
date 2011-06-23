#ifndef FIELDOBJECTS_H
#define FIELDOBJECTS_H

#include "StationaryObject.h"
#include "Self.h"
#include "MobileObject.h"
#include "AmbiguousObject.h"
#include "Tools/FileFormats/TimestampedData.h"
#include <vector>


class FieldObjects: public TimestampedData
{


	public:
            enum StationaryFieldObjectID{
                // 2 Goals: Blue, Yellow + posts
                // Left/Right post is the side when viewing the goals from the front. i.e. on the field.

                FO_BLUE_LEFT_GOALPOST 		= 0,
                FO_BLUE_RIGHT_GOALPOST          = 1,
                FO_YELLOW_LEFT_GOALPOST 	= 2,
                FO_YELLOW_RIGHT_GOALPOST 	= 3,

                // Field Corners
                FO_CORNER_YELLOW_FIELD_LEFT     = 4,
                FO_CORNER_YELLOW_T_LEFT 	= 5,
                FO_CORNER_YELLOW_T_RIGHT 	= 6,
                FO_CORNER_YELLOW_FIELD_RIGHT    = 7,
                FO_CORNER_YELLOW_PEN_LEFT 	= 8,
                FO_CORNER_YELLOW_PEN_RIGHT 	= 9,
                FO_CORNER_CENTRE_T_LEFT 	= 10,
                FO_CORNER_CENTRE_CIRCLE 	= 11,
                FO_CORNER_CENTRE_T_RIGHT 	= 12,
                FO_CORNER_BLUE_PEN_LEFT 	= 13,
                FO_CORNER_BLUE_PEN_RIGHT 	= 14,
                FO_CORNER_BLUE_FIELD_LEFT 	= 15,
                FO_CORNER_BLUE_T_LEFT 		= 16,
                FO_CORNER_BLUE_T_RIGHT          = 17,
                FO_CORNER_BLUE_FIELD_RIGHT 	= 18,

                //Projected Field Corners
                FO_CORNER_PROJECTED_T_YELLOW_LEFT = 19,
                FO_CORNER_PROJECTED_T_YELLOW_RIGHT = 20,
                FO_CORNER_PROJECTED_T_BLUE_LEFT = 21,
                FO_CORNER_PROJECTED_T_BLUE_RIGHT = 22,

                //Penalty Spots
                FO_PENALTY_YELLOW               = 23,
                FO_PENALTY_BLUE                 = 24,

                // Goal Gaps
                //FO_YELLOW_GOAL_GAP 		= 25,
                //FO_BLUE_GOAL_GAP 		= 26,

                NUM_STAT_FIELD_OBJECTS 		= 25

            };

            enum MobileFieldObjectID{

                // Ball and Teammates and Opponents
                FO_BALL 			= 0,
                FO_BLUE_ROBOT_1 		= 1,
                FO_BLUE_ROBOT_2 		= 2,
                FO_BLUE_ROBOT_3			= 3,
                FO_BLUE_ROBOT_4                 = 4,
                FO_PINK_ROBOT_1 		= 5,
                FO_PINK_ROBOT_2 		= 6,
                FO_PINK_ROBOT_3 		= 7,
                FO_PINK_ROBOT_4 		= 8,
                NUM_MOBILE_FIELD_OBJECTS        = 9
            };

            enum AmbiguousObjectID{

                //New unknown Robots
                FO_ROBOT_UNKNOWN 		= 0,
                FO_BLUE_ROBOT_UNKNOWN           = 1,
                FO_PINK_ROBOT_UNKNOWN           = 2,

                // New unknown goal posts
                FO_BLUE_GOALPOST_UNKNOWN 	= 3,
                FO_YELLOW_GOALPOST_UNKNOWN = 4,

                //New Unknown corners
                FO_CORNER_UNKNOWN_INSIDE_L 	= 5,
                FO_CORNER_UNKNOWN_OUTSIDE_L 	= 6,
                FO_CORNER_UNKNOWN_T 	= 7,

                //New Unknown Penalty Spots
                FO_PENALTY_UNKNOWN = 8,

                NUM_AMBIGUOUS_FIELD_OBJECTS = 9
            };
            double m_timestamp;
            Self self;
            vector<StationaryObject> stationaryFieldObjects;
            vector<MobileObject> mobileFieldObjects;
            vector<AmbiguousObject> ambiguousFieldObjects;
            FieldObjects();
            FieldObjects(const FieldObjects& source);
            ~FieldObjects();
    
            void preProcess(const float timestamp);
            void postProcess(const float timestamp);
            double GetTimestamp() const{return m_timestamp;};
            std::string toString(bool visibleOnly=false) const;

    std::vector<FieldObjects::StationaryFieldObjectID> GetPossibleObservationIds(float x, float y, float theta, 
                                                                   float headPan, float headTilt, 
                                                                   float FoV_x, float FoV_y);
    std::vector<FieldObjects::MobileFieldObjectID> GetPossibleMobileObservationIds(float x, float y, float heading,
                                                                                float headYaw, float headPitch,
                                                                                float FoV_x, float FoV_y);

            /*!
            @brief Output streaming operation.
            @param output The output stream.
            @param p_loc The source localisation data to be streamed.
            */
            friend std::ostream& operator<< (std::ostream& output, const FieldObjects& p_mob);

            /*!
            @brief Input streaming operation.
            @param input The input stream.
            @param p_kf The destination localisation data to be streamed to.
            */
            friend std::istream& operator>> (std::istream& input, FieldObjects& p_mob);

	private:
            void InitStationaryFieldObjects();
            void InitMobileFieldObjects();


};

#endif
