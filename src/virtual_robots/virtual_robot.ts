import { Simulator } from './simulator'
import { NUClearNetClient } from '../shared/nuclearnet/nuclearnet_client'
import { Clock } from '../shared/time/clock'
import { FakeNUClearNetClient } from '../server/nuclearnet/fake_nuclearnet_client'
import { DirectNUClearNetClient } from '../server/nuclearnet/direct_nuclearnet_client'
import { NodeSystemClock } from '../server/time/node_clock'
import { flatMap } from './flat_map'

type Opts = {
  fakeNetworking: boolean
  name: string
  simulators: Simulator[]
}

export class VirtualRobot {
  private name: string
  private simulators: Simulator[]

  constructor(private network: NUClearNetClient,
              private clock: Clock,
              opts: { name: string, simulators: Simulator[] }) {
    this.name = opts.name
    this.simulators = opts.simulators
  }

  public static of(opts: Opts): VirtualRobot {
    const network = opts.fakeNetworking ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of()
    const clock = NodeSystemClock
    return new VirtualRobot(network, clock, opts)
  }

  public simulateWithFrequency(frequency: number, index: number, numRobots: number) {
    const disconnect = this.connect()

    const period = 1 / frequency
    const cancelLoop = this.clock.setInterval(() => this.simulate(index, numRobots), period)

    return () => {
      cancelLoop()
      disconnect()
    }
  }

  public send(messageType: string, buffer: Uint8Array, reliable?: boolean) {
    this.network.send({
      type: messageType,
      payload: new Buffer(buffer),
      target: 'nusight',
      reliable,
    })
  }

  public simulate(index: number, numRobots: number) {
    const messages = flatMap(simulator => simulator.simulate(this.clock.now(), index, numRobots), this.simulators)
    messages.forEach(message => this.send(message.messageType, message.buffer))
    return messages
  }

  public connect(): () => void {
    return this.network.connect({ name: this.name })
  }
}
