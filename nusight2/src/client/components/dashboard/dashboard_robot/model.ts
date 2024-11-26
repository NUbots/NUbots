import { computed, observable } from "mobx";

import { Matrix2 } from "../../../../shared/math/matrix2";
import { Matrix3 } from "../../../../shared/math/matrix3";
import { Transform } from "../../../../shared/math/transform";
import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { message } from "../../../../shared/messages";
import { memoize } from "../../../base/memoize";
import { BrowserSystemClock } from "../../../time/browser_clock";
import { RobotModel } from "../../robot/model";

import Mode = message.input.GameState.Data.Mode;
import PenaltyReason = message.input.GameState.Data.PenaltyReason;
import Phase = message.input.GameState.Data.Phase;

export class DashboardRobotModel {
  // Parameters that influence the display
  @observable accessor camera: Transform;
  @observable accessor ballColor: string;
  @observable accessor ballSightColor: string;
  @observable accessor kickTargetColor: string;
  @observable private accessor robot: RobotModel;
  @observable accessor robotColor: string;
  @observable accessor textColor: string;

  // Parameters from the network
  // The timestamp of the last message from the robot (in seconds since an arbitrary time)
  @observable accessor time: number;

  // The player id of the robot, typically 1 through N
  @observable accessor playerId: number;

  // The name of the role the robot is executing
  @observable accessor roleName: string;

  // Battery as a value between 0 and 1 (percentage)
  @observable accessor battery: number;

  // The voltage of the battery
  @observable accessor voltage: number;

  // The robots position and heading and associated covariance
  @observable accessor robotPosition: Vector3; // x,y,theta
  @observable accessor robotPositionCovariance: Matrix3;

  // The position of the ball on the field and associated covariance
  @observable accessor ballPosition: Vector2;
  @observable accessor ballCovariance: Matrix2;

  // The position on the field the robot is kicking towards
  @observable accessor kickTarget: Vector2;

  // The game state information
  @observable accessor gameMode: Mode;
  @observable accessor gamePhase: Phase;
  @observable accessor penaltyReason: PenaltyReason;

  // The timestamp of when we last had an image, saw the ball and saw a goal
  // Measured in seconds compared to the variable `time`
  @observable accessor lastCameraImage: number;
  @observable accessor lastSeenBall: number;
  @observable accessor lastSeenGoal: number;

  // The current walk command
  @observable accessor walkCommand: Vector3;

  constructor(
    robot: RobotModel,
    {
      camera,
      ballColor,
      ballSightColor,
      kickTargetColor,
      robotColor,
      textColor,
      time,
      roleName,
      battery,
      voltage,
      robotPosition,
      robotPositionCovariance,
      ballPosition,
      ballCovariance,
      kickTarget,
      gameMode,
      gamePhase,
      penaltyReason,
      playerId,
      lastCameraImage,
      lastSeenBall,
      lastSeenGoal,
      walkCommand,
    }: DashboardRobotModelOpts,
  ) {
    this.robot = robot;
    this.camera = camera;
    this.ballColor = ballColor;
    this.ballSightColor = ballSightColor;
    this.kickTargetColor = kickTargetColor;
    this.robotColor = robotColor;
    this.textColor = textColor;
    this.time = time;
    this.roleName = roleName;
    this.battery = battery;
    this.voltage = voltage;
    this.robotPosition = robotPosition;
    this.robotPositionCovariance = robotPositionCovariance;
    this.ballPosition = ballPosition;
    this.ballCovariance = ballCovariance;
    this.kickTarget = kickTarget;
    this.gameMode = gameMode;
    this.gamePhase = gamePhase;
    this.penaltyReason = penaltyReason;
    this.playerId = playerId;
    this.lastCameraImage = lastCameraImage;
    this.lastSeenBall = lastSeenBall;
    this.lastSeenGoal = lastSeenGoal;
    this.walkCommand = walkCommand;
  }

  static of = memoize((robot: RobotModel): DashboardRobotModel => {
    return new DashboardRobotModel(robot, {
      ballColor: "#ff9800",
      ballCovariance: Matrix2.of(),
      ballPosition: Vector2.of(),
      ballSightColor: "#4db659",
      battery: -1,
      camera: Transform.of(),
      gameMode: Mode.UNKNOWN_MODE,
      gamePhase: Phase.UNKNOWN_PHASE,
      playerId: -1,
      kickTarget: Vector2.of(),
      kickTargetColor: "#115e2c",
      lastCameraImage: 0,
      lastSeenBall: 0,
      lastSeenGoal: 0,
      penaltyReason: PenaltyReason.UNKNOWN_PENALTY_REASON,
      robotColor: "#015726",
      robotPosition: Vector3.of(),
      robotPositionCovariance: Matrix3.of(),
      roleName: "",
      textColor: "#ffffff",
      time: BrowserSystemClock.now(),
      voltage: -1,
      walkCommand: Vector3.of(),
    });
  });

  @computed
  get connected(): boolean {
    return this.robot.connected;
  }

  @computed
  get id(): string {
    return this.robot.id;
  }

  @computed
  get name(): string {
    return this.robot.name;
  }

  @computed
  get enabled(): boolean {
    return this.robot.enabled;
  }
}

interface DashboardRobotModelOpts {
  camera: Transform;
  ballColor: string;
  ballSightColor: string;
  kickTargetColor: string;
  robotColor: string;
  textColor: string;
  time: number;
  roleName: string;
  battery: number;
  voltage: number;
  robotPosition: Vector3; // x, y, theta
  robotPositionCovariance: Matrix3;
  ballPosition: Vector2;
  ballCovariance: Matrix2;
  kickTarget: Vector2;
  gameMode: Mode;
  gamePhase: Phase;
  penaltyReason: PenaltyReason;
  playerId: number;
  lastCameraImage: number;
  lastSeenBall: number;
  lastSeenGoal: number;
  walkCommand: Vector3;
}
