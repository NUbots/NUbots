#include "Vision.h"
#include "ObjectCandidate.h"
#include "Tools/Math/LSFittedLine.h"

#define GOAL_WIDTH 11 //LAB = 11cm, OFFICIAL = 10cm
#define GOAL_HEIGHT 80 //in cm

#define DISTANCE_BETWEEN_POSTS 140


class GoalDetection
{
  public:
        GoalDetection();
        ~GoalDetection();

        ObjectCandidate FindGoal(std::vector<ObjectCandidate>& FO_Candidates,
                                 std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
			FieldObjects* AllObjects,
                        const std::vector < TransitionSegment > &horizontalSegments,
                        Vision* vision,
                        int height,
                        int width);

        void PostProcessGoalPosts(FieldObjects* AllObjects);


  private:

        void ExtendGoalAboveHorizon(ObjectCandidate* PossibleGoal,
                                    std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
                                    const std::vector < TransitionSegment > &horizontalSegments);

        bool isObjectAPossibleGoal(const ObjectCandidate &PossibleGoal);

        void CheckIsFilled(std::vector< ObjectCandidate >& FO_Candidates, Vision* vision);

        void classifyGoalClosely(ObjectCandidate* PossibleGoal,Vision* vision);

        void CombineOverlappingCandidates(std::vector<ObjectCandidate>& FO_Candidates);

        void CheckCandidateSizeRatio(std::vector<ObjectCandidate>& FO_Candidates,int height,int width);

        bool isCorrectCheckRatio(ObjectCandidate PossibleGoal,int height,int width);

        void CheckCandidateIsInRobot(std::vector<ObjectCandidate>& FO_Candidates, FieldObjects* AllObjects);

        void CheckObjectIsBelowHorizon(std::vector<ObjectCandidate>& FO_Candidates, Vision* vision);

        float FindGoalDistance(const ObjectCandidate &PossibleGoal, Vision* vision);
        float DistanceLineToPoint(const LSFittedLine &midPointLine, const Vector2<int> &point);

        //! SORTING: BIGGEST TO SMALLEST
        void SortObjectCandidates(std::vector<ObjectCandidate>& FO_Candidates);
        static bool ObjectCandidateSizeSortPredicate(const ObjectCandidate& goal1, const ObjectCandidate& goal2);





        unsigned char FindColourOfClosestPost(FieldObjects* AllObjects);


        //! FieldObject Updating Functions
        void UpdateGoalObjects(vector < ObjectCandidate > FO_Candidates, FieldObjects* AllObjects, Vision* vision);

        void UpdateAFieldObject(FieldObjects* AllObjects,Vision* vision, ObjectCandidate* GoalPost ,  int ID, Vector3<float> sphericalPosition);

        void AddAmbiguousGoalPost(ObjectCandidate* GoalPost, FieldObjects* AllObjects, Vision* vision);

        Vector3<float>  CalculateSphericalPosition(ObjectCandidate* GoalPost, Vision* vision);

        float MINIMUM_GOAL_WIDTH_IN_PIXELS;
        float MINIMUM_GOAL_HEIGHT_IN_PIXELS;
};

