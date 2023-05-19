import { observable } from "mobx";

export class RobotModel {
  @observable id: string;
  @observable connected: boolean;
  @observable enabled: boolean;
  @observable name: string;
  @observable address: string;
  @observable port: number;

  constructor({ id, connected, enabled, name, address, port }: RobotModel) {
    this.id = id;
    this.connected = connected;
    this.enabled = enabled;
    this.name = name;
    this.address = address;
    this.port = port;
  }

  static of(opts: RobotModel) {
    return new RobotModel(opts);
  }
}
