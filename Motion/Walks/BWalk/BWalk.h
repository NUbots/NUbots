#ifndef BWALK_H
#define BWALK_H

#include <Vector>
#include <boost/circular_buffer.hpp>

#include "Motion/NUWalk.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/StlVector.h"


enum KickType
{
  none, /**< do not kick */
  left, /**< kick using the left foot */
  right, /**< kick using the right foot */
  sidewardsLeft,
  sidewardsRight
};

typedef std::vector<float> Range;
typedef std::vector<float> Vector2;
typedef std::vector<float> Vector3;
typedef std::vector<float> Pose2D;
typedef Matrix Pose3D;
typedef Matrix RotationMatrix;
typedef Matrix Matrix4x4f;
typedef std::vector<float> Vector4f;
typedef std::vector<float> Vector2f;
typedef boost::circular_buffer<float> RingBufferWithSum;



class BWalk : public NUWalk
{
public:
    BWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~BWalk();
    void doWalk(){return;}

  private:

    class PIDCorrector
    {
    public:
      class Parameters
      {
      public:
        float p;
        float i;
        float d;
        float max;

        Parameters() : p(0.f), i(0.f), d(0.f), max(0.f) {}

        Parameters(float p, float i, float d, float max) : p(p), i(i), d(d), max(max) {}
      };

      PIDCorrector() : sum(0.f), error(0.f) {}

      float getCorrection(float measuredError, float deltaTime, float measurementDelay, const Parameters& parameters)
      {
        float error = -measuredError;
        //int max = std::min(int(measurementDelay / 10.f + 0.5f), corrections.getNumberOfEntries());
        //for(int i = 0; i < max; ++i)
        //error -= corrections.getEntry(i);
        sum += error * deltaTime * parameters.i;
        float correction = error * parameters.p + sum + (error - this->error) / deltaTime * parameters.d;
        this->error = error;
        //corrections.add(correction);
        if(correction > parameters.max)
          correction = parameters.max;
        else if(correction < -parameters.max)
          correction = -parameters.max;
        return correction;
      }

    private:
      float sum;
      float error;
      //RingBuffer<float, 10> corrections;
    };

    class PhaseParameters
    {
    public:
      float start;
      float duration;

      PhaseParameters() : start(0.f), duration(1.f) {}
    };

    /**
    * A collection of walking engine parameters
    */
    class Parameters
    {
    public:
        enum FadeInShape
        {
            sine,
            sqr
        };

        enum MeasurementMode
        {
        torsoMatrix,
        robotModel
        };

      enum ObserverErrorMode
      {
        direct,
        indirect,
        average,
        mixed
      };

      /** Default constructor */
      Parameters()
      {
      }

      float standBikeRefX;
      float standStandbyRefX;
      Vector3 standComPosition;
      float standBodyTilt;
      Vector2 standArmJointAngles;
      int standHardnessAnklePitch;
      int standHardnessAnkleRoll;

      float walkRefX;
      float walkRefXAtFullSpeedX;
      float walkRefY;
      float walkRefYAtFullSpeedX;
      float walkRefYAtFullSpeedY;
      float walkStepDuration;
      float walkStepDurationAtFullSpeedX;
      float walkStepDurationAtFullSpeedY;
      Vector2 walkHeight;
      float walkArmRotation;
      Range walkRefXSoftLimit;
      Range walkRefXLimit;
      Range walkRefYLimit;
      Range walkRefYLimitAtFullSpeedX;
      PhaseParameters walkMovePhase;
      PhaseParameters walkLiftPhase;
      Vector3 walkLiftOffset;
      float walkLiftOffsetJerk;
      Vector3 walkLiftOffsetAtFullSpeedY;
      Vector3 walkLiftRotation;
      Vector3 walkAntiLiftOffset;
      Vector3 walkAntiLiftOffsetAtFullSpeedY;
      float walkComBodyRotation;
      FadeInShape walkFadeInShape;

      Vector3 kickComPosition;
      float kickX0Y;
      Vector2 kickHeight;

      Pose2D speedMax;
      Pose2D speedMaxMin;
      float speedMaxBackwards;
      Pose2D speedMaxChange;

      bool balance;
      Vector3 balanceMinError;
      Vector3 balanceMaxError;
      std::vector<PIDCorrector::Parameters> balanceCom;
      std::vector<PIDCorrector::Parameters> balanceBodyRotation;
      Vector2 balanceStepSize;
      Vector2 balanceStepSizeWhenInstable;
      Vector2 balanceStepSizeWhenPedantic;
      float balanceStepSizeInterpolation;

      float stabilizerOnThreshold;
      float stabilizerOffThreshold;
      int stabilizerDelay;

      MeasurementMode observerMeasurementMode;
      float observerMeasurementDelay;
      ObserverErrorMode observerErrorMode;
      Vector4f observerProcessDeviation;
      Vector2f observerMeasurementDeviation;
      Vector2f observerMeasurementDeviationAtFullSpeedX;
      Vector2f observerMeasurementDeviationWhenInstable;

      bool odometryUseTorsoMatrix;
      Pose2D odometryScale;
      Pose2D odometryUpcomingScale;
      Pose2D odometryUpcomingOffset;

      // computed values
      RotationMatrix standBodyRotation;
      Vector2 kickK;
      Vector2 walkK;
      float te;
      float teAtFullSpeedX;
      float teAtFullSpeedY;

      void computeContants()
      {
        walkK.resize(2,0.0f);
        kickK.resize(2,0.0f);
        const float g = 9806.65f;;
        walkK[0] = sqrt(g / walkHeight[0]);
        walkK[1] = sqrt(g / walkHeight[1]);
        kickK[0] = sqrt(g / kickHeight[0]);
        kickK[1] = sqrt(g / kickHeight[1]);
        te = walkStepDuration * (0.001f * 0.25f);
        teAtFullSpeedX = walkStepDurationAtFullSpeedX * (0.001f * 0.25f);
        teAtFullSpeedY = walkStepDurationAtFullSpeedY * (0.001f * 0.25f);
        //standBodyRotation = RotationMatrix(Vector3(0.f, standBodyTilt, 0.f));
        standBodyRotation = TransformMatrices::RotY(standBodyTilt);
      }
    };

    class LegStance
    {
    public:
      Pose3D leftOriginToFoot;
      Pose3D rightOriginToFoot;
      Vector3 leftOriginToCom;
      Vector3 rightOriginToCom;
    };

    class ArmAndHeadStance
    {
    public:
      float headJointAngles[2];
      float leftArmJointAngles[4];
      float rightArmJointAngles[4];
    };

    class Stance : public LegStance, public ArmAndHeadStance {};

    enum MotionType
    {
      stand,
      standLeft,
      standRight,
      stepping
    };

    enum StepType
    {
      normal,
      unknown,
      fromStand,
      toStand,
      fromStandLeft,
      toStandLeft,
      fromStandRight,
      toStandRight
    };

    class StepSize
    {
    public:
      Vector3 translation;
      float rotation;

      StepSize() : rotation(0.f) {}

      StepSize(float rotation, float x, float y) : rotation(rotation)
      {
          translation = Vector3(3,0.0f);
          translation[0] = x;
          translation[1] = y;
      }

      StepSize(const StepSize& other) : translation(other.translation), rotation(other.rotation) {}

      StepSize& operator+=(const StepSize& other)
      {
          translation[0] += other.translation[0];
          translation[1] += other.translation[1];
          translation[2] += other.translation[2];
        rotation += other.rotation;
        return *this;
      }

      StepSize& operator-=(const StepSize& other)
      {
          translation[0] -= other.translation[0];
          translation[1] -= other.translation[1];
          translation[2] -= other.translation[2];
        rotation -= other.rotation;
        return *this;
      }

      StepSize& operator*=(float factor)
      {
          translation[0] *= factor;
          translation[1] *= factor;
          translation[2] *= factor;
        rotation *= factor;
        return *this;
      }

      StepSize operator+(const StepSize& other) const
      {
        return StepSize(*this) += other;
      }

      StepSize operator-(const StepSize& other) const
      {
        return StepSize(*this) -= other;
      }

      StepSize operator*(float factor) const
      {
        return StepSize(*this) *= factor;
      }

      operator Pose2D() const
      {
          Pose2D result = Pose2D(3, 0.0f);
          result[0] = rotation;
          result[1] = translation[0];
          result[0] = translation[1];
          return result;
      }
    };

    class PendulumParameters
    {
    public:
      Vector2 x0; /**< pendulum position at t = 0 */
      Vector2 xv0; /**< pendulum velocity at t = 0 */
      Vector2 r; /**< origin to ref */
      Vector2 c;
      Vector2 k; /**< sqrt(g / h) */
      float te; /**< end of pendulum phase */
      float tb; /**< begin of pendulum phase */
      StepSize s;
      Vector3 l; /**< lift offset */
      Vector3 lRotation; /**< lift rotation */
      Vector3 al; /**< anti lift offset */
      StepType type;
      Vector2 xtb; /**< pendulum position at t = tb */
      Vector2 xvtb; /**< pendulum velocity at t = tb */
      Range sXLimit;
      Range rXLimit;
      Range rYLimit;
      KickType kickType;
      float originalRX;
      PendulumParameters() : type(unknown) {}
    };

    enum SupportLeg
    {
      left,
      right
    };

    class PendulumPlayer : public PendulumParameters
    {
    public:
      BWalk* walkingEngine;
      SupportLeg supportLeg;
      bool active;
      bool launching;
      float t; /**< current time */
      PendulumParameters next;

      PendulumPlayer() : walkingEngine(0), active(false) {}

      void seek(float deltaT);
      inline bool isActive() const {return active;}
      inline bool isLaunching() const {return launching;}
      void getStance(LegStance& stance, float* leftArmAngle, float* rightArmAngle, StepSize* stepOffset) const;

      float smoothShape(float r) const;

    protected:
      void generateNextStepSize();
      void computeSwapTimes(float t, float xt, float xvt, float errory);
      void computeRefZmp(float t, float xt, float xvt, float errorx);
    };

    class ObservedPendulumPlayer : public PendulumPlayer
    {
    public:
      void init(StepType stepType, float t, SupportLeg supportLeg, const Vector2& r, const Vector2& x0, const Vector2& k, float deltaTime);
      void applyCorrection(const Vector3& leftError, const Vector3& rightError, float deltaTime);

    private:
      Matrix4x4f cov;
    };
/*
    class KickPlayer
    {
    public:
      KickPlayer();

      void init(KickType type, const Vector2& ballPosition, const Vector2& target);
      void seek(float deltaT);
      float getLength() const;
      float getCurrentPosition() const;
      void apply(Stance& stance);
      void stop() {kick = 0;};
      inline bool isActive() const {return kick ? true : false;}
      inline KickType getType() const {return type;}
      void setParameters(const Vector2& ballPosition, const Vector2& target);

      inline bool isKickMirrored(KickType type) const {return (type - 1) % 2 != 0;}
      bool isKickStandKick(KickType type) const;
      void getKickStepSize(KickType type, float& rotation, Vector3& translation) const;
      void getKickPreStepSize(KickType type, float& rotation, Vector3& translation) const;
      float getKickDuration(KickType type) const;
      float getKickRefX(KickType type, float defaultValue) const;

    private:
      WalkingEngineKick* kick;
      bool mirrored;
      KickType type;

      WalkingEngineKick kicks[(numOfKickTypes - 1) / 2];
    };
    */

    Parameters p; /**< The walking engine parameters */

    void init();

    /**
    * The central update method to generate the walking motion
    * @param walkingEngineOutput The WalkingEngineOutput (mainly the resulting joint angles)
    */
    void update();
    bool emergencyShutOff;
    MotionType currentMotionType;
    float currentRefX;
    Stance targetStance;
    ObservedPendulumPlayer observedPendulumPlayer;
    PendulumPlayer pendulumPlayer;
    //KickPlayer kickPlayer;

    void updateMotionRequest();
    MotionType requestedMotionType;
    Pose2D requestedWalkTarget;
    Vector2 balanceStepSize;

    void updateKickPlayer();

    void updateObservedPendulumPlayer();

    void computeMeasuredStance();
    Vector3 measuredLeftToCom;
    Vector3 measuredRightToCom;

    void computeExpectedStance();
    Vector3 expectedLeftToCom;
    Vector3 expectedRightToCom;
    StepSize stepOffset;

    void computeError();
    Vector3 leftError;
    Vector3 rightError;
//    RingBufferWithSum<float, 100> instability;
    RingBufferWithSum instability;
    bool instable;
    unsigned beginOfStable;

    void updatePendulumPlayer();

    void generateTargetStance();
    boost::circular_buffer<LegStance> legStances;
    void getStandStance(LegStance& stance) const;

    void generateJointRequest();
    PIDCorrector leftControllerX, leftControllerY, leftControllerZ;
    PIDCorrector rightControllerX, rightControllerY, rightControllerZ;
    PIDCorrector bodyControllerX, bodyControllerY;
    Vector3 bodyToCom;
    Vector3 lastAverageComToAnkle;

    void generateOutput();
    void generateDummyOutput();

    void generateNextStepSize(SupportLeg nextSupportLeg, StepType lastStepType, KickType lastKickType, PendulumParameters& next);
    SupportLeg lastNextSupportLeg;
    PendulumParameters nextPendulumParameters;
    Pose2D lastSelectedSpeed;
    KickType lastExecutedWalkingKick;

    void computeOdometryOffset();
    Pose2D odometryOffset;
    Pose3D lastTorsoMatrix;
    SupportLeg lastSupportLeg;
    StepSize lastStepOffset;
    Pose2D upcomingOdometryOffset;
    bool upcomingOdometryOffsetValid;
};

#endif // BWALK_H
