import { action } from "mobx";
import * as THREE from "three";

import { Matrix4 } from "../../../shared/math/matrix4";
import { Vector3 } from "../../../shared/math/vector3";
import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

export class KinematicsNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Sensors);
  }

  static of(nusightNetwork: NUsightNetwork): KinematicsNetwork {
    const network = Network.of(nusightNetwork);
    return new KinematicsNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };
}
