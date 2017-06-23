import { NUClearNet } from 'nuclearnet.js'
import { Clock } from '../../src/server/time/clock'
import { CancelTimer } from '../../src/server/time/node_clock'
import { flatMap } from './flat_map'
import { Simulator } from './simulator'
import { NodeSystemClock } from '../server/time/node_clock'
import { FakeNUClearNet } from './nuclearnet/fake_nuclearnet'

export class RobotSimulator {
  private name: string
  private simulators: Simulator[]
  public messagesSent: number

  public constructor(private network: NUClearNet,
                     private clock: Clock,
                     opts: { name: string, simulators: Simulator[] }) {
    this.name = opts.name
    this.simulators = opts.simulators

    this.messagesSent = 0
  }

  public static of(opts: { fakeNetworking: boolean, name: string; simulators: Simulator[] }): RobotSimulator {
    const network = opts.fakeNetworking ? FakeNUClearNet.of() : new NUClearNet()
    const clock = NodeSystemClock
    return new RobotSimulator(network, clock, opts)
  }

  public simulateWithFrequency(frequency: number) {
    this.connect()

    const period = 1000 / frequency
    const cancelLoop = this.clock.setInterval(() => this.simulate(), period)

    return () => {
      cancelLoop()
      this.disconnect()
    }
  }

  public send(messageType: string, buffer: Uint8Array, reliable?: boolean) {
    const header = new Buffer(9)
    header.writeUInt8(0, 0)
    // TODO: A 64bit timestamp used to be written to the header here, but it does not seem to be used?
    this.network.send({
      type: `NUsight<${messageType}>`,
      payload: Buffer.concat([header, new Buffer(buffer)]),
      target: 'nusight',
      reliable,
    })
    this.messagesSent++
  }

  private simulate() {
    const messages = flatMap(simulator => simulator.simulate(this.clock.now()), this.simulators)
    messages.forEach(message => this.send(message.messageType, message.buffer))
  }

  private connect() {
    this.network.connect({ name: this.name })
  }

  private disconnect() {
    this.network.disconnect()
  }
}

export class SimulatorStatus {
  private lastMessagesSent: number

  public constructor(private clock: Clock,
                     private simulator: RobotSimulator) {
    this.lastMessagesSent = 0
  }

  public static of(simulator: RobotSimulator): SimulatorStatus {
    return new SimulatorStatus(NodeSystemClock, simulator)
  }

  public statusEvery(seconds: number): CancelTimer {
    return this.clock.setInterval(() => {
      const messagesSent = this.simulator.messagesSent
      const delta = messagesSent - this.lastMessagesSent
      // tslint:disable-next-line no-console
      console.log(`Simulator: Sending ${(delta / seconds).toFixed(2)} messages/second.`)
      this.lastMessagesSent = messagesSent
    }, seconds * 1000)
  }
}
