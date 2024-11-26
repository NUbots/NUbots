import { observable } from "mobx";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";

export class OdometryVisualizerModel {
  @observable.ref accessor Hwt: Matrix4;
  @observable.ref accessor accelerometer: Vector3;
  @observable.ref accessor camera: OdometryCamera;

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
  @observable.ref accessor distance: number;
  @observable.ref accessor pitch: number;
  @observable.ref accessor yaw: number;

  constructor({ distance, pitch, yaw }: { distance: number; pitch: number; yaw: number }) {
    this.distance = distance;
    this.pitch = pitch;
    this.yaw = yaw;
  }

  static of({ distance }: { distance: number }) {
    return new OdometryCamera({ distance, pitch: 0, yaw: 0 });
  }
}
