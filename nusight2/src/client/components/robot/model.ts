import { observable } from "mobx";

import { NUClearNetPeerWithType } from "../../../shared/nuclearnet/nuclearnet_client";

export class RobotModel {
  @observable accessor id: string;
  @observable accessor connected: boolean;
  @observable accessor type: NUClearNetPeerWithType["type"];
  @observable accessor enabled: boolean;
  @observable accessor name: string;
  @observable accessor address: string;
  @observable accessor port: number;

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
