import { observable } from "mobx";

import { NUClearNetPeerWithType } from "../../../shared/nuclearnet/nuclearnet_client";

export class RobotModel {
  @observable id: string;
  @observable connected: boolean;
  @observable type: NUClearNetPeerWithType["type"];
  @observable enabled: boolean;
  @observable name: string;
  @observable address: string;
  @observable port: number;

  constructor({ id, connected, type, enabled, name, address, port }: RobotModel) {
    this.id = id;
    this.connected = connected;
    this.type = type;
    this.enabled = enabled;
    this.name = name;
    this.address = address;
    this.port = port;
  }

  static of(opts: RobotModel) {
    return new RobotModel(opts);
  }
}
