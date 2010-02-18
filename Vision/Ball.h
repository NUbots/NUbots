#include "Vision.h"
#include "ObjectCandidate.h"
#include "CircleFitting.h"

class Ball
{
  public:
	Ball();
        ~Ball();

        Circle FindBall(std::vector<ObjectCandidate> FO_Candidates,
			FieldObjects* AllObjects,
                        Vision* vision,
                        int height,
                        int width);
  private:
  	bool isObjectAPossibleBall(ObjectCandidate PossibleBall);
        std::vector < Vector2<int> > classifyBallClosely(ObjectCandidate PossibleBall,Vision* vision,int height,int width);
        bool isCorrectCheckRatio(ObjectCandidate PossibleBall,int height,int width);
        Circle isCorrectFit(std::vector < Vector2<int> > ballPoints, ObjectCandidate PossibleBall);
	
};

