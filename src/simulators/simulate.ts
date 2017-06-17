import * as minimist from 'minimist'
import { SimulatorStatus } from './robot_simulator'
import { RobotSimulator } from './robot_simulator'
import { SensorDataSimulator } from './sensor_data_simulator'
import { Simulator } from './simulator'

function main() {
  const args = minimist(process.argv.slice(2))

  const simulators = getSimulators(args)
  const robotSimulator = RobotSimulator.of({
    name: 'Robot Simulator',
    simulators,
  })
  SimulatorStatus.of(robotSimulator).statusEvery(2)
  robotSimulator.simulateWithFrequency(60)
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
