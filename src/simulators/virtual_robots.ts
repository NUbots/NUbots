import { range } from '../shared/base/range'
import { Simulator } from './simulator'
import { VirtualRobot } from './virtual_robot'

type Opts = {
  fakeNetworking: boolean
  numRobots: number
  simulators: Simulator[]
}

export class VirtualRobots {
  private robots: VirtualRobot[]

  public constructor(opts: { robots: VirtualRobot[] }) {
    this.robots = opts.robots
  }

  public static of(opts: Opts): VirtualRobots {
    const robots = range(opts.numRobots).map(index => VirtualRobot.of({
      fakeNetworking: opts.fakeNetworking,
      name: `Virtual Robot #${index + 1}`,
      simulators: opts.simulators,
    }))
    return new VirtualRobots({ robots })
  }

  public simulateWithFrequency(frequency: number): () => void {
    const stops = this.robots.map((robot, index) => robot.simulateWithFrequency(frequency, index, this.robots.length))
    return () => stops.forEach(stop => stop())
  }

  public simulate(): void {
    this.robots.forEach((robot, index) => robot.simulate(index, this.robots.length))
  }

  public connect(): void {
    this.robots.forEach(robot => robot.connect())
  }
}
