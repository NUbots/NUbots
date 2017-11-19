import * as minimist from 'minimist'
import { OverviewSimulator } from './simulators/overview_simulator'
import { SensorDataSimulator } from './simulators/sensor_data_simulator'
import { SimulatorOpts } from './virtual_robot'
import { VirtualRobots } from './virtual_robots'

function main() {
  const args = minimist(process.argv.slice(2))

  const simulators = getSimulators(args)
  const virtualRobots = VirtualRobots.of({
    fakeNetworking: false,
    numRobots: 6,
    simulators,
  })
  virtualRobots.startSimulators()
}

function getSimulators(args: minimist.ParsedArgs): SimulatorOpts[] {
  const simulators = []
  if (args.overview || args.all) {
    simulators.push({ frequency: 1, simulator: OverviewSimulator.of() })
  }
  if (args.sensors || args.all) {
    simulators.push({ frequency: 60, simulator: SensorDataSimulator.of() })
  }
  if (simulators.length === 0) {
    // If no simulators given, enable them all.
    return getSimulators({ ...args, all: true })
  }
  return simulators
}

if (require.main === module) {
  main()
}
