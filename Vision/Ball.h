
#include "ObjectCandidate.h"
#include "CircleFitting.h"
class Vision;
class FieldObjects;

class Ball
{
  public:
        Ball();
        ~Ball();

        Circle FindBall(std::vector<ObjectCandidate> FO_Candidates, FieldObjects* AllObjects, Vision* vision, int height, int width);
  private:
        bool isObjectAPossibleBall(const ObjectCandidate &PossibleBall);
        bool isObjectInRobot(const ObjectCandidate &PossibleBall, FieldObjects* AllObjects);
        bool isObjectTooBig(const ObjectCandidate &PossibleBall, Vision* vision);
        float getMaxPixelsOfBall(Vision* vision);
        std::vector < Vector2<int> > classifyBallClosely(ObjectCandidate &PossibleBall,Vision* vision,int height,int width, float &pinkPercentage);
        std::vector < Vector2<int> > scanLeft(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision);
        std::vector < Vector2<int> > scanRight(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision);
        std::vector < Vector2<int> > scanDown(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision);
        std::vector < Vector2<int> > scanUp(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision);
        bool isCorrectCheckRatio(const ObjectCandidate &PossibleBall,int height,int width);
        Circle isCorrectFit(const std::vector < Vector2<int> > &ballPoints, const ObjectCandidate &PossibleBall, Vision* vision, float &pinkPercentage);
        bool isVisualBallDistanceCloseDistanceToPoint(Circle circ, Vision* vision,const ObjectCandidate &PossibleBall, FieldObjects* AllObjects);
private:
    std::vector<unsigned char> m_ball_colours;
};

