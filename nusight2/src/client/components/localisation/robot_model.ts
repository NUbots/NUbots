import { observable } from "mobx";
import { computed } from "mobx";

import { Matrix4 } from "../../../shared/math/matrix4";
import { Quaternion } from "../../../shared/math/quaternion";
import { Vector3 } from "../../../shared/math/vector3";
import { memoize } from "../../base/memoize";
import { RobotModel } from "../robot/model";

class ServoMotor {
  @observable angle: number;

  constructor({ angle }: ServoMotor) {
    this.angle = angle;
  }

  static of() {
    return new ServoMotor({ angle: 0 });
  }
}

export class ServoMotorSet {
  @observable rightShoulderPitch: ServoMotor;
  @observable leftShoulderPitch: ServoMotor;
  @observable rightShoulderRoll: ServoMotor;
  @observable leftShoulderRoll: ServoMotor;
  @observable rightElbow: ServoMotor;
  @observable leftElbow: ServoMotor;
  @observable rightHipYaw: ServoMotor;
  @observable leftHipYaw: ServoMotor;
  @observable rightHipRoll: ServoMotor;
  @observable leftHipRoll: ServoMotor;
  @observable rightHipPitch: ServoMotor;
  @observable leftHipPitch: ServoMotor;
  @observable rightKnee: ServoMotor;
  @observable leftKnee: ServoMotor;
  @observable rightAnklePitch: ServoMotor;
  @observable leftAnklePitch: ServoMotor;
  @observable rightAnkleRoll: ServoMotor;
  @observable leftAnkleRoll: ServoMotor;
  @observable headPan: ServoMotor;
  @observable headTilt: ServoMotor;

  constructor({
    rightShoulderPitch,
    leftShoulderPitch,
    rightShoulderRoll,
    leftShoulderRoll,
    rightElbow,
    leftElbow,
    rightHipYaw,
    leftHipYaw,
    rightHipRoll,
    leftHipRoll,
    rightHipPitch,
    leftHipPitch,
    rightKnee,
    leftKnee,
    rightAnklePitch,
    leftAnklePitch,
    rightAnkleRoll,
    leftAnkleRoll,
    headPan,
    headTilt,
  }: ServoMotorSet) {
    this.rightShoulderPitch = rightShoulderPitch;
    this.leftShoulderPitch = leftShoulderPitch;
    this.rightShoulderRoll = rightShoulderRoll;
    this.leftShoulderRoll = leftShoulderRoll;
    this.rightElbow = rightElbow;
    this.leftElbow = leftElbow;
    this.rightHipYaw = rightHipYaw;
    this.leftHipYaw = leftHipYaw;
    this.rightHipRoll = rightHipRoll;
    this.leftHipRoll = leftHipRoll;
    this.rightHipPitch = rightHipPitch;
    this.leftHipPitch = leftHipPitch;
    this.rightKnee = rightKnee;
    this.leftKnee = leftKnee;
    this.rightAnklePitch = rightAnklePitch;
    this.leftAnklePitch = leftAnklePitch;
    this.rightAnkleRoll = rightAnkleRoll;
    this.leftAnkleRoll = leftAnkleRoll;
    this.headPan = headPan;
    this.headTilt = headTilt;
  }

  static of() {
    return new ServoMotorSet({
      rightShoulderPitch: ServoMotor.of(),
      leftShoulderPitch: ServoMotor.of(),
      rightShoulderRoll: ServoMotor.of(),
      leftShoulderRoll: ServoMotor.of(),
      rightElbow: ServoMotor.of(),
      leftElbow: ServoMotor.of(),
      rightHipYaw: ServoMotor.of(),
      leftHipYaw: ServoMotor.of(),
      rightHipRoll: ServoMotor.of(),
      leftHipRoll: ServoMotor.of(),
      rightHipPitch: ServoMotor.of(),
      leftHipPitch: ServoMotor.of(),
      rightKnee: ServoMotor.of(),
      leftKnee: ServoMotor.of(),
      rightAnklePitch: ServoMotor.of(),
      leftAnklePitch: ServoMotor.of(),
      rightAnkleRoll: ServoMotor.of(),
      leftAnkleRoll: ServoMotor.of(),
      headPan: ServoMotor.of(),
      headTilt: ServoMotor.of(),
    });
  }
}

export class FieldIntersection {
  @observable type: string;
  @observable position: Vector3;
  constructor({ type, position }: { type: string; position: Vector3 }) {
    this.type = type;
    this.position = position;
  }
}

export class LocalisationRobotModel {
  @observable private model: RobotModel;
  @observable name: string;
  @observable color?: string;
  @observable Htw: Matrix4; // World to torso
  @observable Hfw: Matrix4; // World to field
  @observable Rwt: Quaternion; // Torso to world rotation.
  @observable motors: ServoMotorSet;
  @observable fieldLinePoints: { rPWw: Vector3[] };
  @observable particles: { particle: Vector3[] }; // Particle filter particles.
  @observable ball?: { rBWw: Vector3 };
  @observable fieldIntersections?: FieldIntersection[];
  // Both bottom and top points of goal are in world space.
  @observable goals: { points: { bottom: Vector3; top: Vector3 }[] };
  @observable robots: { id: number; rRWw: Vector3 }[];

  constructor({
    model,
    name,
    color,
    Htw,
    Hfw,
    Rwt,
    motors,
    fieldLinePoints,
    particles,
    ball,
    fieldIntersections,
    goals,
    robots,
  }: {
    model: RobotModel;
    name: string;
    color?: string;
    Htw: Matrix4;
    Hfw: Matrix4;
    Rwt: Quaternion;
    motors: ServoMotorSet;
    fieldLinePoints: { rPWw: Vector3[] };
    particles: { particle: Vector3[] };
    ball?: { rBWw: Vector3 };
    fieldIntersections?: FieldIntersection[];
    goals: { points: { bottom: Vector3; top: Vector3 }[] };
    robots: { id: number; rRWw: Vector3 }[];
  }) {
    this.model = model;
    this.name = name;
    this.color = color;
    this.Htw = Htw;
    this.Hfw = Hfw;
    this.Rwt = Rwt;
    this.motors = motors;
    this.fieldLinePoints = fieldLinePoints;
    this.particles = particles;
    this.ball = ball;
    this.fieldIntersections = fieldIntersections;
    this.goals = goals;
    this.robots = robots;
  }

  static of = memoize((model: RobotModel): LocalisationRobotModel => {
    return new LocalisationRobotModel({
      model,
      name: model.name,
      Htw: Matrix4.of(),
      Hfw: Matrix4.of(),
      Rwt: Quaternion.of(),
      motors: ServoMotorSet.of(),
      fieldLinePoints: { rPWw: [] },
      particles: { particle: [] },
      goals: { points: [] },
      robots: [],
    });
  });

  @computed get id() {
    return this.model.id;
  }

  @computed get visible() {
    return this.model.enabled;
  }

  /** Torso to field transformation */
  @computed
  get Hft(): Matrix4 {
    return this.Hfw.multiply(this.Htw.invert());
  }

  /** Field line points in field space */
  @computed
  get rPFf(): Vector3[] {
    return this.fieldLinePoints.rPWw.map((rPWw) => rPWw.applyMatrix4(this.Hfw));
  }

  /** Ball position in field space */
  @computed
  get rBFf(): Vector3 | undefined {
    return this.ball?.rBWw.applyMatrix4(this.Hfw);
  }

  /** Goal positions in field space */
  @computed
  get rGFf(): { bottom: Vector3; top: Vector3 }[] {
    return this.goals?.points.map((pair) => ({
      bottom: pair?.bottom.applyMatrix4(this.Hfw),
      top: pair?.top.applyMatrix4(this.Hfw),
    }));
  }

  /** Robot positions in field space */
  @computed
  get rRFf(): Vector3[] {
    return this.robots?.map((robot) => robot.rRWw.applyMatrix4(this.Hfw));
  }

  /** Field intersections in field space */
  @computed
  get fieldIntersectionsF(): FieldIntersection[] | undefined {
    return this.fieldIntersections?.map((intersection) => {
      return new FieldIntersection({
        type: intersection.type,
        position: intersection.position.applyMatrix4(this.Hfw),
      });
    });
  }
}
