import { computed } from 'mobx'
import { observable } from 'mobx'
import { message } from '../../../../shared/proto/messages'
import { memoize } from '../../../base/memoize'
import { Transform } from '../../../math/transform'
import { Vector2 } from '../../../math/vector2'
import { RobotModel } from '../../robot/model'
import State = message.behaviour.Behaviour.State

import Mode = message.input.GameState.Data.Mode
import PenaltyReason = message.input.GameState.Data.PenaltyReason
import Phase = message.input.GameState.Data.Phase

export class DashboardRobotModel {
  @observable public camera: Transform
  @observable public battery: number
  @observable public ballColor: string
  @observable public ballPosition: Vector2
  @observable public ballSightColor: string
  @observable public ballWorldPosition: Vector2
  @observable public behaviourState: State
  @observable public gameMode: Mode
  @observable public gamePhase: Phase
  @observable public id: number
  @observable public kickTarget: Vector2
  @observable public kickTargetColor: string
  @observable public lastCameraImage: number
  @observable public lastSeenBall: number
  @observable public lastSeenGoal: number
  @observable public lastSeenObstacle: number
  @observable public penaltyReason: PenaltyReason
  @observable private robot: RobotModel
  @observable public robotBorderColor: string
  @observable public robotColor: string
  @observable public robotHeading: Vector2
  @observable public robotPosition: Vector2
  @observable public textColor: string
  @observable public time: number
  @observable public voltage: number

  constructor(robot: RobotModel, opts: Partial<DashboardRobotModel>) {
    this.robot = robot
    Object.assign(this, opts)
  }

  public static of = memoize((robot: RobotModel): DashboardRobotModel => {
    return new DashboardRobotModel(robot, {
      battery: -1,
      ballColor: '#ff9800',
      ballPosition: Vector2.of(),
      ballSightColor: '#4DB6AC',
      ballWorldPosition: Vector2.of(),
      behaviourState: State.UNKNOWN,
      gameMode: Mode.UNKNOWN_MODE,
      gamePhase: Phase.UNKNOWN_PHASE,
      id: -1,
      kickTarget: Vector2.of(),
      kickTargetColor: '#00796B',
      lastCameraImage: 0,
      lastSeenBall: 0,
      lastSeenGoal: 0,
      lastSeenObstacle: 0,
      penaltyReason: PenaltyReason.UNKNOWN_PENALTY_REASON,
      robotBorderColor: 'transparent',
      robotColor: '#015457',
      robotHeading: Vector2.of(),
      robotPosition: Vector2.of(),
      textColor: '#fff',
      time: Date.now() / 1000,
      voltage: -1,
    })
  })

  @computed
  public get name(): string {
    return this.robot.name
  }

  @computed
  public get visible(): boolean {
    return this.robot.enabled
  }
}
