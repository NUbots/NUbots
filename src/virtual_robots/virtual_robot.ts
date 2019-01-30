import { DirectNUClearNetClient } from '../server/nuclearnet/direct_nuclearnet_client'
import { FakeNUClearNetClient } from '../server/nuclearnet/fake_nuclearnet_client'
import { NodeSystemClock } from '../server/time/node_clock'
import { NUClearNetClient } from '../shared/nuclearnet/nuclearnet_client'
import { Clock } from '../shared/time/clock'

import { flatMap } from './flat_map'
import { PeriodicSimulator, Simulator } from './simulator'

type Opts = {
  fakeNetworking: boolean
  name: string
  simulators: Simulator[],
  periodicSimulators: PeriodicSimulatorOpts[]
}

export type PeriodicSimulatorOpts = {
  frequency: number,
  simulator: PeriodicSimulator
}

export class VirtualRobot {
  private name: string
  private simulators: Simulator[]
  private periodicSimulators: PeriodicSimulatorOpts[]

  constructor(private network: NUClearNetClient,
              private clock: Clock,
              opts: { name: string, simulators: Simulator[], periodicSimulators: PeriodicSimulatorOpts[] }) {
    this.name = opts.name
    this.simulators = opts.simulators
    this.periodicSimulators = opts.periodicSimulators
  }

  static of(opts: Opts): VirtualRobot {
    const network = opts.fakeNetworking ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of()
    const clock = NodeSystemClock
    return new VirtualRobot(network, clock, opts)
  }

  startSimulators(index: number, numRobots: number) {
    const disconnect = this.connect()

    const stops: Array<() => void> = this.periodicSimulators
      .map(opts => {
        const period = 1 / opts.frequency
        return this.clock.setInterval(() => this.simulate(opts.simulator, index, numRobots), period)
      })
      .concat(this.simulators.map(simulator => {
        return simulator.start(this.network)
      }))

    return () => {
      stops.forEach(stop => stop())
      disconnect()
    }
  }

  send(messageType: string, buffer: Uint8Array, reliable?: boolean) {
    this.network.send({
      type: messageType,
      payload: Buffer.from(buffer),
      target: 'nusight',
      reliable,
    })
  }

  simulateAll(index: number, numRobots: number) {
    const messages = flatMap(
      opts => opts.simulator.simulate(this.clock.now(), index, numRobots),
      this.periodicSimulators,
    )
    messages.forEach(message => this.send(message.messageType, message.buffer))
    return messages
  }

  connect(): () => void {
    return this.network.connect({ name: this.name })
  }

  private simulate(simulator: PeriodicSimulator, index: number, numRobots: number) {
    const messages = simulator.simulate(this.clock.now(), index, numRobots)
    messages.forEach(message => this.send(message.messageType, message.buffer))
    return messages
  }
}
