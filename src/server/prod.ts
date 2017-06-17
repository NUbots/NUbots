import * as compression from 'compression'
import * as history from 'connect-history-api-fallback'
import * as express from 'express'
import * as http from 'http'
import * as minimist from 'minimist'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'
import { FakeNUClearNet } from '../simulators/nuclearnet/fake_nuclearnet'
import { RobotSimulator } from '../simulators/robot_simulator'
import { SensorDataSimulator } from '../simulators/sensor_data_simulator'
import { NodeSystemClock } from './time/node_clock'

const args = minimist(process.argv.slice(2))
const withSimulators = args['with-simulators'] || false

const app = express()
const server = http.createServer(app)
sio(server)

const root = `${__dirname}/../../build`
app.use(history())
app.use(compression())
app.use(express.static(root))
app.use(favicon(`${__dirname}/../assets/favicon.ico`))

const port = process.env.PORT || 9090
server.listen(port, () => {
  /* tslint:disable no-console */
  console.log(`NUsight server started at http://localhost:${port}`)
})

if (withSimulators) {
  const robotSimulator = new RobotSimulator({
    name: 'Sensors Simulator',
    network: FakeNUClearNet.of(),
    clock: NodeSystemClock,
    simulators: [
      SensorDataSimulator.of(),
    ],
  })
  robotSimulator.simulateWithFrequency(60)
}
