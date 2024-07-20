import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { TempMonitorRobotModel } from "./model";

export class TempMonitorNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Sensors, this.onSensors);
  }

  static of(nusightNetwork: NUsightNetwork): TempMonitorNetwork {
    const network = Network.of(nusightNetwork);
    return new TempMonitorNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onSensors(robotModel: RobotModel, packet: message.input.Sensors) {
    const robot = TempMonitorRobotModel.of(robotModel);
    packet.servo.forEach((servo) => {
      robot.servoTemperatures.set(servo.id!, servo.temperature!);
    });
  }
}
