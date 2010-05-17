#include "Vision.h"
#include "ObjectCandidate.h"
#include "Tools/Math/LSFittedLine.h"

#define GOAL_WIDTH 11 //LAB = 11cm, OFFICIAL = 10cm
#define GOAL_HEIGHT 80 //in cm


class GoalDetection
{
  public:
        GoalDetection();
        ~GoalDetection();

        ObjectCandidate FindGoal(std::vector<ObjectCandidate>& FO_Candidates,
                                 std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
			FieldObjects* AllObjects,
                        std::vector < TransitionSegment > horizontalSegments,
                        Vision* vision,
                        int height,
                        int width);


  private:

        void ExtendGoalAboveHorizon(ObjectCandidate* PossibleGoal,
                                    std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
                                    std::vector < TransitionSegment > horizontalSegments);

  	bool isObjectAPossibleGoal(ObjectCandidate PossibleGoal);

        void classifyGoalClosely(ObjectCandidate* PossibleGoal,Vision* vision);

        void CombineOverlappingCandidates(std::vector<ObjectCandidate>& FO_Candidates);

        void CheckCandidateSizeRatio(std::vector<ObjectCandidate>& FO_Candidates,int height,int width);

        bool isCorrectCheckRatio(ObjectCandidate PossibleGoal,int height,int width);

        float FindGoalDistance(ObjectCandidate PossibleGoal, Vision* vision);
        float DistanceLineToPoint(LSFittedLine midPointLine, Vector2<int> point);

        //! SORTING: BIGGEST TO SMALLEST
        void SortObjectCandidates(std::vector<ObjectCandidate>& FO_Candidates);
        static bool ObjectCandidateSizeSortPredicate(const ObjectCandidate& goal1, const ObjectCandidate& goal2);

        void UpdateAFieldObject(FieldObjects* AllObjects,Vision* vision, ObjectCandidate& GoalPost ,  int ID);

        float MINIMUM_GOAL_WIDTH_IN_PIXELS;
        float MINIMUM_GOAL_HEIGHT_IN_PIXELS;
};

