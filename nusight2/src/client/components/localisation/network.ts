import { action } from "mobx";
import * as THREE from "three";

import { Matrix4 } from "../../../shared/math/matrix4";
import { Quaternion } from "../../../shared/math/quaternion";
import { Vector3 } from "../../../shared/math/vector3";
import { message } from "../../../shared/messages";
import { Imat4 } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { LocalisationModel } from "./model";
import { LocalisationRobotModel } from "./robot_model";
import { FieldIntersection } from "./robot_model";

export class LocalisationNetwork {
  constructor(private network: Network, private model: LocalisationModel) {
    this.network.on(message.input.Sensors, this.onSensors);
    this.network.on(message.localisation.Field, this.onField);
    this.network.on(message.vision.FieldLines, this.onFieldLines);
    this.network.on(message.localisation.Ball, this.onBall);
    this.network.on(message.vision.FieldIntersections, this.onFieldIntersections);
  }

  static of(nusightNetwork: NUsightNetwork, model: LocalisationModel): LocalisationNetwork {
    const network = Network.of(nusightNetwork);
    return new LocalisationNetwork(network, model);
  }

  destroy() {
    this.network.off();
  }

  @action
  private onField = (robotModel: RobotModel, field: message.localisation.Field) => {
    const robot = LocalisationRobotModel.of(robotModel);
    robot.Hfw = Matrix4.from(field.Hfw);
  };

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
  private onFieldIntersections(robotModel: RobotModel, fieldIntersections: message.vision.FieldIntersections) {
    const robot = LocalisationRobotModel.of(robotModel);

    robot.fieldIntersections = fieldIntersections.intersections.map((intersection) => {
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
    robot.Rwt = new Quaternion(Rwt.x, Rwt.y, Rwt.z, Rwt.w);

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
