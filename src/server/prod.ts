import * as compression from 'compression'
import * as history from 'connect-history-api-fallback'
import * as express from 'express'
import * as http from 'http'
import * as minimist from 'minimist'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'
import { RobotSimulator } from '../simulators/robot_simulator'
import { SensorDataSimulator } from '../simulators/sensor_data_simulator'
import { WebSocketProxyNUClearNetServer } from './nuclearnet/web_socket_proxy_nuclearnet_server'
import { WebSocketServer } from './nuclearnet/web_socket_server'

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

if (withSimulators) {
  const robotSimulator = RobotSimulator.of({
    fakeNetworking: true,
    name: 'Sensors Simulator',
    simulators: [
      SensorDataSimulator.of(),
    ],
  })
  robotSimulator.simulateWithFrequency(60)
}

WebSocketProxyNUClearNetServer.of(WebSocketServer.of(sioNetwork.of('/nuclearnet')), {
  fakeNetworking: withSimulators,
})
