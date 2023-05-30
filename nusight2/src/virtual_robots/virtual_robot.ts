import { NUClearNetClient } from "../shared/nuclearnet/nuclearnet_client";

import { Simulator } from "./simulator";

export class VirtualRobot {
  constructor(
    private readonly name: string,
    private readonly nuclearnetClient: NUClearNetClient,
    private readonly nuclearnetAddress: string,
    private readonly simulators: Simulator[],
  ) {
    this.nuclearnetClient.connect({ name: this.name, address: nuclearnetAddress });
  }

  static of({
    name,
    simulators,
    nuclearnetClient,
    nuclearnetAddress,
  }: {
    name: string;
    simulators: Simulator[];
    nuclearnetClient: NUClearNetClient;
    nuclearnetAddress: string;
  }) {
    return new VirtualRobot(name, nuclearnetClient, nuclearnetAddress, simulators);
  }

  start(): () => void {
    const stops = this.simulators.map((simulator) => simulator.start());
    return () => stops.forEach((stop) => stop());
  }
}
