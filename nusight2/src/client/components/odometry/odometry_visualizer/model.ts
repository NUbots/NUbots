import { observable } from "mobx";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";

export class OdometryVisualizerModel {
  @observable.ref Hwt: Matrix4;
  @observable.ref accelerometer: Vector3;
  @observable.ref camera: OdometryCamera;

  constructor({ Hwt, accelerometer, camera }: { Hwt: Matrix4; accelerometer: Vector3; camera: OdometryCamera }) {
    this.Hwt = Hwt;
    this.accelerometer = accelerometer;
    this.camera = camera;
  }

  static of({ Hwt = Matrix4.of(), accelerometer }: { Hwt?: Matrix4; accelerometer: Vector3 }) {
    return new OdometryVisualizerModel({
      Hwt,
      accelerometer,
      camera: OdometryCamera.of({ distance: 2 }),
    });
  }
}

export class OdometryCamera {
  @observable.ref distance: number;
  @observable.ref pitch: number;
  @observable.ref yaw: number;

  constructor({ distance, pitch, yaw }: { distance: number; pitch: number; yaw: number }) {
    this.distance = distance;
    this.pitch = pitch;
    this.yaw = yaw;
  }

  static of({ distance }: { distance: number }) {
    return new OdometryCamera({ distance, pitch: 0, yaw: 0 });
  }
}
