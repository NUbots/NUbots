import * as compression from 'compression'
import * as history from 'connect-history-api-fallback'
import * as express from 'express'
import * as http from 'http'
import * as minimist from 'minimist'
import { NUClearNet } from 'nuclearnet.js'
import 'reflect-metadata'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'
import { RobotSimulator } from '../simulators/robot_simulator'
import { SensorDataSimulator } from '../simulators/sensor_data_simulator'
import { NUSightServer } from './app/server'
import { getContainer } from './inversify.config'
import { ClockType } from './time/clock'
import { Clock } from './time/clock'

const args = minimist(process.argv.slice(2))
const withSimulators = args['with-simulators'] || false

const app = express()
const server = http.createServer(app)
const sioNetwork = sio(server)

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

const container = getContainer({ fakeNetworking: withSimulators })

if (withSimulators) {
  const robotSimulator = new RobotSimulator(
    container.get<NUClearNet>(NUClearNet),
    container.get<Clock>(ClockType),
    {
      name: 'Sensors Simulator',
      simulators: [
        SensorDataSimulator.of(),
      ],
    },
  )
  robotSimulator.simulateWithFrequency(60)
}

new NUSightServer(container.get<NUClearNet>(NUClearNet), sioNetwork).connect()
