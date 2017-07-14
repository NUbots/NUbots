import * as minimist from 'minimist'
import { VirtualRobots } from './virtual_robots'
import { SensorDataSimulator } from './sensor_data_simulator'
import { Simulator } from './simulator'

function main() {
  const args = minimist(process.argv.slice(2))

  const simulators = getSimulators(args)
  const virtualRobots = VirtualRobots.of({
    fakeNetworking: false,
    numRobots: 1,
    simulators,
  })
  virtualRobots.simulateWithFrequency(60)
}

function getSimulators(args: minimist.ParsedArgs): Simulator[] {
  const simulators = []
  if (args.sensors || args.all) {
    simulators.push(SensorDataSimulator.of())
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
