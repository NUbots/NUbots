import { range } from '../shared/base/range'
import { VirtualRobot } from './virtual_robot'
import { SimulatorOpts } from './virtual_robot'

type Opts = {
  fakeNetworking: boolean
  numRobots: number
  simulators: SimulatorOpts[]
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

  public startSimulators(): () => void {
    const stops = this.robots.map((robot, index) => robot.startSimulators(index, this.robots.length))
    return () => stops.forEach(stop => stop())
  }

  public simulateAll(): void {
    this.robots.forEach((robot, index) => robot.simulateAll(index, this.robots.length))
  }

  public connect(): void {
    this.robots.forEach(robot => robot.connect())
  }
}
