import { action } from "mobx";
import * as THREE from "three";

import { Matrix4 } from "../../../shared/math/matrix4";
import { Vector3 } from "../../../shared/math/vector3";
import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { OdometryRobotModel } from "./model";

export class OdometryNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Sensors, this.onSensors);
  }

  static of(nusightNetwork: NUsightNetwork): OdometryNetwork {
    const network = Network.of(nusightNetwork);
    return new OdometryNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onSensors(robotModel: RobotModel, packet: message.input.Sensors) {
    const robot = OdometryRobotModel.of(robotModel);
    robot.visualizerModel.Hwt = Matrix4.fromThree(
      new THREE.Matrix4().copy(Matrix4.from(packet.Htw).toThree()).invert(),
    );
    robot.visualizerModel.accelerometer = Vector3.from(packet.accelerometer);
  }
}
