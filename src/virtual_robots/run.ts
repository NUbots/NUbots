import * as minimist from 'minimist'

import { Simulator } from './simulator'
import { OverviewSimulator } from './simulators/overview_simulator'
import { ScriptDataSimulator } from './simulators/script_data_simulator'
import { SensorDataSimulator } from './simulators/sensor_data_simulator'
import { PeriodicSimulatorOpts } from './virtual_robot'
import { VirtualRobots } from './virtual_robots'

function main() {
  const args = minimist(process.argv.slice(2))

  const { simulators, periodicSimulators } = getSimulators(args)
  const virtualRobots = VirtualRobots.of({
    fakeNetworking: false,
    numRobots: 6,
    simulators,
    periodicSimulators,
  })
  virtualRobots.startSimulators()
}

function getSimulators(args: minimist.ParsedArgs): {
  simulators: Simulator[],
  periodicSimulators: PeriodicSimulatorOpts[]
} {
  const simulators = []
  const periodicSimulators = []

  if (args.overview || args.all) {
    periodicSimulators.push({ frequency: 1, simulator: OverviewSimulator.of() })
  }
  if (args.sensors || args.all) {
    periodicSimulators.push({ frequency: 60, simulator: SensorDataSimulator.of() })
  }
  if (args.scripts || args.all) {
    simulators.push(ScriptDataSimulator.of())
  }
  if (periodicSimulators.length === 0 && simulators.length === 0) {
    // If no simulators given, enable them all.
    return getSimulators({ ...args, all: true })
  }
  return {
    simulators,
    periodicSimulators,
  }
}

if (require.main === module) {
  main()
}
