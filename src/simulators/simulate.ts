import * as minimist from 'minimist'
import { NUClearNet } from 'nuclearnet.js'
import 'reflect-metadata'
import { getContainer } from '../server/inversify.config'
import { Clock } from '../server/time/clock'
import { ClockType } from '../server/time/clock'
import { SimulatorStatus } from './robot_simulator'
import { RobotSimulator } from './robot_simulator'
import { SensorDataSimulator } from './sensor_data_simulator'
import { Simulator } from './simulator'

function main() {
  const args = minimist(process.argv.slice(2))

  const simulators = getSimulators(args)
  const container = getContainer({ fakeNetworking: false })
  const clock = container.get<Clock>(ClockType)
  const robotSimulator = new RobotSimulator(
    container.get<NUClearNet>(NUClearNet),
    clock,
    {
      name: 'Robot Simulator',
      simulators,
    },
  )
  new SimulatorStatus(clock, robotSimulator).statusEvery(2)
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
