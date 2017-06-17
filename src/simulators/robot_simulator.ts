import { NUClearNet } from 'nuclearnet.js'
import { Clock } from '../../src/server/time/clock'
import { NodeSystemClock } from '../../src/server/time/node_clock'
import { CancelTimer } from '../../src/server/time/node_clock'
import { flatMap } from './flat_map'
import { Simulator } from './simulator'

interface RobotSimulatorOpts {
  name: string
  simulators: Simulator[]
}

export class RobotSimulator {
  public messagesSent: number
  private name: string
  private clock: Clock
  private network: NUClearNet
  private simulators: Simulator[]

  public constructor(opts: { name: string, clock: Clock, network: NUClearNet, simulators: Simulator[] }) {
    Object.assign(this, opts)

    this.messagesSent = 0
  }

  public static of({ name, simulators }: RobotSimulatorOpts) {
    const network = new NUClearNet()
    const clock = NodeSystemClock
    return new RobotSimulator({ name, clock, network, simulators })
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

  public constructor(private simulator: RobotSimulator,
                     private clock: Clock) {
    this.lastMessagesSent = 0
  }

  public static of(simulator: RobotSimulator) {
    const clock = NodeSystemClock
    return new SimulatorStatus(simulator, clock)
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
