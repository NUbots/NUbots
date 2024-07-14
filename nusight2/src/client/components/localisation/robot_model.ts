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

export class BoundingBox {
  @observable minX: number;
  @observable minY: number;
  @observable maxX: number;
  @observable maxY: number;

  constructor({ minX, minY, maxX, maxY }: { minX: number; minY: number; maxX: number; maxY: number }) {
    this.minX = minX;
    this.minY = minY;
    this.maxX = maxX;
    this.maxY = maxY;
  }
}

export class LocalisationRobotModel {
  @observable private model: RobotModel;
  @observable name: string;
  @observable color: string;
  @observable Htw: Matrix4; // World to torso
  @observable Hrw: Matrix4; // World to robot
  @observable Hfw: Matrix4; // World to field
  @observable Hrd?: Matrix4; // Walk path desired pose in robot space.
  @observable Rwt: Quaternion; // Torso to world rotation.
  @observable motors: ServoMotorSet;
  @observable fieldLinePoints: { rPWw: Vector3[] };
  @observable particles: { particle: Vector3[] }; // Particle filter particles.
  @observable ball?: { rBWw: Vector3 };
  @observable fieldIntersections?: FieldIntersection[];
  // Both bottom and top points of goal are in world space.
  @observable goals: { points: { bottom: Vector3; top: Vector3 }[] };
  @observable robots: { id: number; rRWw: Vector3 }[];
  @observable purpose: string;
  @observable max_align_radius: number;
  @observable min_align_radius: number;
  @observable angle_to_final_heading: number;
  @observable angle_to_target: number;
  @observable translational_error: number;
  @observable min_angle_error: number;
  @observable max_angle_error: number;
  @observable velocity_target: Vector3;
  @observable boundingBox?: BoundingBox;
  @observable player_id: number;

  constructor({
    model,
    name,
    color,
    Htw,
    Hrw,
    Hfw,
    Hrd,
    Rwt,
    motors,
    fieldLinePoints,
    particles,
    ball,
    fieldIntersections,
    goals,
    robots,
    purpose,
    max_align_radius,
    min_align_radius,
    angle_to_final_heading,
    angle_to_target,
    translational_error,
    min_angle_error,
    max_angle_error,
    velocity_target,
    boundingBox,
    player_id,
  }: {
    model: RobotModel;
    name: string;
    color: string;
    Htw: Matrix4;
    Hrw: Matrix4;
    Hfw: Matrix4;
    Hrd?: Matrix4;
    Rwt: Quaternion;
    motors: ServoMotorSet;
    fieldLinePoints: { rPWw: Vector3[] };
    particles: { particle: Vector3[] };
    ball?: { rBWw: Vector3 };
    fieldIntersections?: FieldIntersection[];
    goals: { points: { bottom: Vector3; top: Vector3 }[] };
    robots: { id: number; rRWw: Vector3 }[];
    purpose: string;
    max_align_radius: number;
    min_align_radius: number;
    angle_to_final_heading: number;
    angle_to_target: number;
    translational_error: number;
    min_angle_error: number;
    max_angle_error: number;
    velocity_target: Vector3;
    boundingBox?: BoundingBox;
    player_id: number;
  }) {
    this.model = model;
    this.name = name;
    this.color = color;
    this.Htw = Htw;
    this.Hrw = Hrw;
    this.Hfw = Hfw;
    this.Hrd = Hrd;
    this.Rwt = Rwt;
    this.motors = motors;
    this.fieldLinePoints = fieldLinePoints;
    this.particles = particles;
    this.ball = ball;
    this.fieldIntersections = fieldIntersections;
    this.goals = goals;
    this.robots = robots;
    this.purpose = purpose;
    this.max_align_radius = max_align_radius;
    this.min_align_radius = min_align_radius;
    this.angle_to_final_heading = angle_to_final_heading;
    this.angle_to_target = angle_to_target;
    this.translational_error = translational_error;
    this.min_angle_error = min_angle_error;
    this.max_angle_error = max_angle_error;
    this.velocity_target = velocity_target;
    this.boundingBox = boundingBox;
    this.player_id = player_id;
  }

  static of = memoize((model: RobotModel): LocalisationRobotModel => {
    return new LocalisationRobotModel({
      model,
      name: model.name,
      color: "black",
      Htw: Matrix4.of(),
      Hrw: Matrix4.of(),
      Hfw: Matrix4.of(),
      Rwt: Quaternion.of(),
      motors: ServoMotorSet.of(),
      fieldLinePoints: { rPWw: [] },
      particles: { particle: [] },
      goals: { points: [] },
      robots: [],
      purpose: "",
      max_align_radius: 0,
      min_align_radius: 0,
      angle_to_final_heading: 0,
      angle_to_target: 0,
      translational_error: 0,
      min_angle_error: 0,
      max_angle_error: 0,
      velocity_target: Vector3.of(),
      player_id: -1,
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

  /** Transform from robot space to field space */
  @computed
  get Hfr(): Matrix4 | undefined {
    return this.Hfw.multiply(this.Hrw.invert());
  }

  /** Walk path goal pose in field space */
  @computed
  get Hfd(): Matrix4 | undefined {
    if (!this.Hfr || !this.Hrd) {
      return Matrix4.of();
    }
    return this.Hfr.multiply(this.Hrd);
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
