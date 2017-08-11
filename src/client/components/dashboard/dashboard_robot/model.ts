import { computed } from 'mobx'
import { observable } from 'mobx'
import { message } from '../../../../shared/proto/messages'
import { memoize } from '../../../base/memoize'
import { Matrix2 } from '../../../math/matrix2'
import { Matrix3 } from '../../../math/matrix3'
import { Transform } from '../../../math/transform'
import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'
import { BrowserSystemClock } from '../../../time/browser_clock'
import { RobotModel } from '../../robot/model'
import State = message.behaviour.Behaviour.State
import Mode = message.input.GameState.Data.Mode
import PenaltyReason = message.input.GameState.Data.PenaltyReason
import Phase = message.input.GameState.Data.Phase

export class DashboardRobotModel {

  // Parameters that influence the display
  @observable public camera: Transform
  @observable public ballColor: string
  @observable public ballSightColor: string
  @observable public kickTargetColor: string
  @observable private robot: RobotModel
  @observable public robotBorderColor: string
  @observable public robotColor: string
  @observable public textColor: string

  // Parameters from the network
  // The timestamp of the last message from the robot (in seconds since an arbitrary time)
  @observable public time: number

  // The player id of the robot, typically 1 through N
  @observable public playerId: number

  // The name of the role the robot is executing
  @observable public roleName: string

  // Battery as a value between 0 and 1 (percentage)
  @observable public battery: number

  // The voltage of the battery
  @observable public voltage: number

  // The state of the behaviour as an enum
  @observable public behaviourState: State

  // The robots position and heading and associated covariance
  @observable public robotPosition: Vector3 // x,y,theta
  @observable public robotPositionCovariance: Matrix3

  // The position of the ball on the field and associated covariance
  @observable public ballPosition: Vector2
  @observable public ballCovariance: Matrix2

  // The position on the field the robot is kicking towards
  @observable public kickTarget: Vector2

  // The game state information
  @observable public gameMode: Mode
  @observable public gamePhase: Phase
  @observable public penaltyReason: PenaltyReason

  // The timestamp of when we last had an image, saw the ball and saw a goal
  // Measured in seconds compared to the variable `time`
  @observable public lastCameraImage: number
  @observable public lastSeenBall: number
  @observable public lastSeenGoal: number

  // The walk plan and the current walk command
  @observable public walkPathPlan: Vector2[]
  @observable public walkCommand: Vector3

  constructor(robot: RobotModel, opts: DashboardRobotModelOpts) {
    this.robot = robot
    Object.assign(this, opts)
  }

  public static of = memoize((robot: RobotModel): DashboardRobotModel => {
    return new DashboardRobotModel(robot, {
      ballColor: '#ff9800',
      ballCovariance: Matrix2.of(),
      ballPosition: Vector2.of(),
      ballSightColor: '#4DB6AC',
      battery: -1,
      behaviourState: State.UNKNOWN,
      camera: Transform.of(),
      gameMode: Mode.UNKNOWN_MODE,
      gamePhase: Phase.UNKNOWN_PHASE,
      playerId: -1,
      kickTarget: Vector2.of(),
      kickTargetColor: '#00796B',
      lastCameraImage: 0,
      lastSeenBall: 0,
      lastSeenGoal: 0,
      penaltyReason: PenaltyReason.UNKNOWN_PENALTY_REASON,
      robotBorderColor: 'transparent',
      robotColor: '#015457',
      robotPosition: Vector3.of(),
      robotPositionCovariance: Matrix3.of(),
      roleName: '',
      textColor: '#fff',
      time: BrowserSystemClock.now(),
      voltage: -1,
      walkCommand: Vector3.of(),
      walkPathPlan: [],
    })
  })

  @computed
  public get connected(): boolean {
    return this.robot.connected
  }

  @computed
  public get id(): string {
    return this.robot.id
  }

  @computed
  public get name(): string {
    return this.robot.name
  }

  @computed
  public get enabled(): boolean {
    return this.robot.enabled
  }
}

interface DashboardRobotModelOpts {
  camera: Transform
  ballColor: string
  ballSightColor: string
  kickTargetColor: string
  robotBorderColor: string
  robotColor: string
  textColor: string
  time: number
  roleName: string
  battery: number
  voltage: number
  behaviourState: State
  robotPosition: Vector3 // x, y, theta
  robotPositionCovariance: Matrix3
  ballPosition: Vector2
  ballCovariance: Matrix2
  kickTarget: Vector2
  gameMode: Mode
  gamePhase: Phase
  penaltyReason: PenaltyReason
  playerId: number
  lastCameraImage: number
  lastSeenBall: number
  lastSeenGoal: number
  walkPathPlan: Vector2[]
  walkCommand: Vector3
}
