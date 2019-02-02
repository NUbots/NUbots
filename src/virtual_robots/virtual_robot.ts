import { NUClearNetClient } from '../shared/nuclearnet/nuclearnet_client'

import { Simulator } from './simulator'

export class VirtualRobot {
  constructor(
    private readonly name: string,
    private readonly nuclearnetClient: NUClearNetClient,
    private readonly simulators: Simulator[],
  ) {
    this.nuclearnetClient.connect({ name: this.name })
  }

  static of({ name, nuclearnetClient, simulators }: {
    name: string,
    simulators: Simulator[],
    nuclearnetClient: NUClearNetClient
  }) {
    return new VirtualRobot(name, nuclearnetClient, simulators)
  }

  start(): () => void {
    const stops = this.simulators.map(simulator => simulator.start())
    return () => stops.forEach(stop => stop())
  }
}
