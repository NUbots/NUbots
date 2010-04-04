#include "Vision.h"
#include "ObjectCandidate.h"

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
                                    bool* usedAboveHorizonCandidate,
                                    std::vector < TransitionSegment > horizontalSegments);

  	bool isObjectAPossibleGoal(ObjectCandidate PossibleGoal);

        void classifyGoalClosely(ObjectCandidate* PossibleBall,Vision* vision,int height,int width);

        bool isCorrectCheckRatio(ObjectCandidate PossibleGoal,int height,int width);

        float FindGoalDistance(ObjectCandidate PossibleGoal, Vision* vision);
};

