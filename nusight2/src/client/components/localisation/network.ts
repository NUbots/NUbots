import { action } from "mobx";
import * as THREE from "three";

import { Matrix2 } from "../../../shared/math/matrix2";
import { Matrix3 } from "../../../shared/math/matrix3";
import { Matrix4 } from "../../../shared/math/matrix4";
import { Quaternion } from "../../../shared/math/quaternion";
import { Vector2 } from "../../../shared/math/vector2";
import { Vector3 } from "../../../shared/math/vector3";
import { message } from "../../../shared/messages";
import { Imat4 } from "../../../shared/messages";
import { TimestampObject } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { DashboardRobotModel } from "./dashboard_components/dashboard_robot/model";
import { LocalisationModel } from "./model";
import { LocalisationRobotModel } from "./robot_model";
import { FieldIntersection } from "./robot_model";

export class LocalisationNetwork {
  constructor(
    private network: Network,
    private model: LocalisationModel,
  ) {
    this.network.on(message.input.Sensors, this.onSensors);
    this.network.on(message.localisation.Field, this.onField);
    this.network.on(message.localisation.Ball, this.onBall);
    this.network.on(message.localisation.Robots, this.onRobots);
    this.network.on(message.vision.FieldLines, this.onFieldLines);
    this.network.on(message.vision.FieldIntersections, this.onFieldIntersections);
    this.network.on(message.vision.Goals, this.onGoals);
    this.network.on(message.planning.WalkToDebug, this.onWalkToDebug);
    this.network.on(message.vision.FieldIntersections, this.onFieldIntersections);
    this.network.on(message.strategy.WalkInsideBoundedBox, this.WalkInsideBoundedBox);
    this.network.on(message.purpose.Purpose, this.onPurpose);
    this.network.on(message.behaviour.state.WalkState, this.onWalkState);
    this.network.on(message.support.nusight.Overview, this.onOverview);
    this.network.on(message.input.StellaMap, this.onStellaMap);
  }

  static of(nusightNetwork: NUsightNetwork, model: LocalisationModel): LocalisationNetwork {
    const network = Network.of(nusightNetwork);
    return new LocalisationNetwork(network, model);
  }

  destroy() {
    this.network.off();
  }

  // Reverse lookup for protobuf enums
  getKey(enumType: any, enumValue: number) {
    return Object.keys(enumType).find((key) => enumType[key] === enumValue);
  }

  @action
  private onField = (robotModel: RobotModel, field: message.localisation.Field) => {
    const robot = LocalisationRobotModel.of(robotModel);

    // Flip the field if the robot is on the red team
    robot.Hfw =
      robot.teamColour === "red"
        ? (robot.Hfw = Matrix4.fromRotationZ(Math.PI).multiply(Matrix4.from(field.Hfw)))
        : Matrix4.from(field.Hfw);

    robot.particles = field.particles.map((particle) => Vector3.from(particle));
    robot.associationLines = field.associationLines.map((line) => ({
      start: Vector3.from(line.start),
      end: Vector3.from(line.end),
    }));
  };

  @action
  private onWalkToDebug = (robotModel: RobotModel, walkToDebug: message.planning.WalkToDebug) => {
    const robot = LocalisationRobotModel.of(robotModel);
    robot.Hrd = Matrix4.from(walkToDebug.Hrd);
    robot.maxAlignRadius = walkToDebug.maxAlignRadius;
    robot.minAlignRadius = walkToDebug.minAlignRadius;
    robot.angleToFinalHeading = walkToDebug.angleToFinalHeading;
    robot.angleToTarget = walkToDebug.angleToTarget;
    robot.translationalError = walkToDebug.translationalError;
    robot.minAngleError = walkToDebug.minAngleError;
    robot.maxAngleError = walkToDebug.maxAngleError;
    robot.velocityTarget = Vector3.from(walkToDebug.velocityTarget);
  };

  @action.bound
  private WalkInsideBoundedBox(robotModel: RobotModel, boundedBox: message.strategy.WalkInsideBoundedBox) {
    const robot = LocalisationRobotModel.of(robotModel);
    robot.boundingBox = {
      minX: boundedBox.xMin,
      maxX: boundedBox.xMax,
      minY: boundedBox.yMin,
      maxY: boundedBox.yMax,
    };
  }

  @action.bound
  private onPurpose(robotModel: RobotModel, purpose: message.purpose.Purpose) {
    const robot = LocalisationRobotModel.of(robotModel);
    const position = purpose.purpose;
    robot.purpose = this.getKey(message.purpose.SoccerPosition, position!)!;

    robot.playerId = purpose.playerId!;

    // Update colour based on player id
    if (robot.playerId === 1) {
      robot.color = "blue";
    } else if (robot.playerId === 2) {
      robot.color = "purple";
    } else if (robot.playerId === 3) {
      robot.color = "red";
    } else if (robot.playerId === 4) {
      robot.color = "orange";
    } else if (robot.playerId === 5) {
      robot.color = "yellow";
    } else {
      robot.color = "black";
    }

    robot.teamColour = purpose.teamColour == message.input.GameState.TeamColour.RED ? "red" : "blue";
  }

  @action.bound
  private onFieldLines(robotModel: RobotModel, fieldLines: message.vision.FieldLines) {
    const robot = LocalisationRobotModel.of(robotModel);
    robot.fieldLinePoints.rPWw = fieldLines.rPWw.map((rPWw) => Vector3.from(rPWw));
  }

  @action.bound
  private onBall(robotModel: RobotModel, ball: message.localisation.Ball) {
    const robot = LocalisationRobotModel.of(robotModel);
    robot.ball = { rBWw: Vector3.from(ball.rBWw) };
  }

  @action.bound
  private onRobots(robotModel: RobotModel, localisation_robots: message.localisation.Robots) {
    const robot = LocalisationRobotModel.of(robotModel);
    robot.robots = localisation_robots.robots.map((localisation_robot) => {
      return {
        id: localisation_robot.id!,
        rRWw: Vector3.from(localisation_robot.rRWw),
        color: localisation_robot.teammate ? robot.teamColour : robot.teamColour === "red" ? "blue" : "red",
      };
    });
  }

  @action.bound
  private onFieldIntersections(robotModel: RobotModel, fieldIntersections: message.vision.FieldIntersections) {
    const robot = LocalisationRobotModel.of(robotModel);

    robot.rIWw = fieldIntersections.intersections.map((intersection) => {
      let intersection_type = "";
      if (intersection.type === 0) {
        intersection_type = "UNKNOWN";
      } else if (intersection.type === 1) {
        intersection_type = "L_INTERSECTION";
      } else if (intersection.type === 2) {
        intersection_type = "T_INTERSECTION";
      } else if (intersection.type === 3) {
        intersection_type = "X_INTERSECTION";
      }

      return new FieldIntersection({ type: intersection_type, position: Vector3.from(intersection.rIWw) });
    });
  }

  @action.bound
  private onGoals(robotModel: RobotModel, goalsMessage: message.vision.Goals) {
    const { Hcw, goals } = goalsMessage;
    const Hwc = Matrix4.from(Hcw).invert();
    const robot = LocalisationRobotModel.of(robotModel);
    robot.goals.points = goals.map((goal) => ({
      bottom: Vector3.from(goal.post?.bottom).multiplyScalar(goal.post!.distance!).applyMatrix4(Hwc),
      top: Vector3.from(goal.post?.top).multiplyScalar(goal.post!.distance!).applyMatrix4(Hwc),
    }));
  }

  @action
  private onSensors = (robotModel: RobotModel, sensors: message.input.Sensors) => {
    // Ignore empty Sensors packets which may be emitted by the nbs scrubber
    // when there's no Sensors data at a requested timestamp).
    if (!sensors.Htw) {
      return;
    }

    const robot = LocalisationRobotModel.of(robotModel);

    const { rotation: Rwt } = decompose(new THREE.Matrix4().copy(fromProtoMat44(sensors.Htw!)).invert());
    robot.Htw = Matrix4.from(sensors.Htw);
    robot.Hrw = Matrix4.from(sensors.Hrw);
    robot.Rwt = new Quaternion(Rwt.x, Rwt.y, Rwt.z, Rwt.w);

    // Store Hwn transform from sensors message
    if (sensors.Hwn) {
      robot.Hwn = Matrix4.from(sensors.Hwn);
    }
    console.log("Hwn translation", robot.Hwn.decompose().translation);

    robot.motors.rightShoulderPitch.angle = sensors.servo[0].presentPosition!;
    robot.motors.leftShoulderPitch.angle = sensors.servo[1].presentPosition!;
    robot.motors.rightShoulderRoll.angle = sensors.servo[2].presentPosition!;
    robot.motors.leftShoulderRoll.angle = sensors.servo[3].presentPosition!;
    robot.motors.rightElbow.angle = sensors.servo[4].presentPosition!;
    robot.motors.leftElbow.angle = sensors.servo[5].presentPosition!;
    robot.motors.rightHipYaw.angle = sensors.servo[6].presentPosition!;
    robot.motors.leftHipYaw.angle = sensors.servo[7].presentPosition!;
    robot.motors.rightHipRoll.angle = sensors.servo[8].presentPosition!;
    robot.motors.leftHipRoll.angle = sensors.servo[9].presentPosition!;
    robot.motors.rightHipPitch.angle = sensors.servo[10].presentPosition!;
    robot.motors.leftHipPitch.angle = sensors.servo[11].presentPosition!;
    robot.motors.rightKnee.angle = sensors.servo[12].presentPosition!;
    robot.motors.leftKnee.angle = sensors.servo[13].presentPosition!;
    robot.motors.rightAnklePitch.angle = sensors.servo[14].presentPosition!;
    robot.motors.leftAnklePitch.angle = sensors.servo[15].presentPosition!;
    robot.motors.rightAnkleRoll.angle = sensors.servo[16].presentPosition!;
    robot.motors.leftAnkleRoll.angle = sensors.servo[17].presentPosition!;
    robot.motors.headPan.angle = sensors.servo[18].presentPosition!;
    robot.motors.headTilt.angle = sensors.servo[19].presentPosition!;
  };

  @action.bound
  private onStellaMap(robotModel: RobotModel, stella: message.input.StellaMap) {
    const robot = LocalisationRobotModel.of(robotModel);

    console.log("Stella map points", stella.rPWwMap.length);

    // Update Stella map points
    if (stella.rPWwMap && stella.rPWwMap.length > 0) {
      robot.stellaMapPoints.rPWw_map = stella.rPWwMap.map((point) => Vector3.from(point));
      robot.stellaMapPoints.rPWw_ground = stella.rPWwGround.map((point) => Vector3.from(point));
      robot.stellaMapPoints.rPWw_scale = stella.rPWwScale.map((point) => Vector3.from(point));
      robot.stellaMapPoints.rPWw_unscaled = stella.rPWwUnscaled.map((point) => Vector3.from(point));
    }
  }

  @action.bound
  private onWalkState(robotModel: RobotModel, walk_state: message.behaviour.state.WalkState) {
    const robot = LocalisationRobotModel.of(robotModel);

    // If phase changed, add current trajectories to history before updating
    if (robot.walkPhase !== walk_state.phase && robot.torsoTrajectory.length > 0) {
      robot.addToTrajectoryHistory(robot.torsoTrajectoryF, robot.swingFootTrajectoryF);
    }

    // Update current state
    robot.torsoTrajectory = walk_state.torsoTrajectory.map((pose) => Matrix4.from(pose));
    robot.swingFootTrajectory = walk_state.swingFootTrajectory.map((pose) => Matrix4.from(pose));
    robot.Hwp = Matrix4.from(walk_state.Hwp);
    robot.walkPhase = walk_state.phase;
  }

  @action
  private onOverview = (robotModel: RobotModel, overview: message.support.nusight.Overview) => {
    const robot = DashboardRobotModel.of(robotModel);

    // Timestamp this message was sent (for comparison with last seen)
    robot.time = TimestampObject.toSeconds(overview.timestamp);

    // The id number of the robot
    robot.playerId = overview.robotId;

    // Name of the executing binary
    robot.roleName = overview.roleName;

    // Battery as a value between 0 and 1 (percentage)
    robot.battery = overview.battery;

    // Voltage (in volts!)
    robot.voltage = overview.voltage;

    // The position of the robot on the field in field coordinates
    // overview.robotPosition is a 3 sized vector (x,y,heading)
    robot.robotPosition = Vector3.from(overview.robotPosition);
    robot.robotPositionCovariance = Matrix3.from(overview.robotPositionCovariance);

    // The position of the ball in field coordinates
    robot.ballPosition = Vector2.from(overview.ballPosition);
    robot.ballCovariance = Matrix2.from(overview.ballPositionCovariance);

    // The location on the field the robot wants to kick in field coordinates
    robot.kickTarget = Vector2.from(overview.kickTarget);

    // The game mode the robot thinks it is
    robot.gameMode = overview.gameMode;
    robot.gamePhase = overview.gamePhase;
    robot.penaltyReason = overview.penaltyReason;

    // The last time we had a camera image, saw a ball/goal
    robot.lastCameraImage = TimestampObject.toSeconds(overview.lastCameraImage);
    robot.lastSeenBall = TimestampObject.toSeconds(overview.lastSeenBall);
    robot.lastSeenGoal = TimestampObject.toSeconds(overview.lastSeenGoal);

    // The walk command and
    robot.walkCommand = Vector3.from(overview.walkCommand);
  };
}

function decompose(m: THREE.Matrix4): {
  translation: THREE.Vector3;
  rotation: THREE.Quaternion;
  scale: THREE.Vector3;
} {
  const translation = new THREE.Vector3();
  const rotation = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  m.decompose(translation, rotation, scale);
  return { translation, rotation, scale };
}

function fromProtoMat44(m: Imat4): THREE.Matrix4 {
  return new THREE.Matrix4().set(
    m!.x!.x!,
    m!.y!.x!,
    m!.z!.x!,
    m!.t!.x!,
    m!.x!.y!,
    m!.y!.y!,
    m!.z!.y!,
    m!.t!.y!,
    m!.x!.z!,
    m!.y!.z!,
    m!.z!.z!,
    m!.t!.z!,
    m!.x!.t!,
    m!.y!.t!,
    m!.z!.t!,
    m!.t!.t!,
  );
}
