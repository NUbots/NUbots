import * as compression from 'compression'
import * as history from 'connect-history-api-fallback'
import * as express from 'express'
import * as http from 'http'
import * as minimist from 'minimist'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'

import * as NUClearNetProxyParser from '../shared/nuclearnet/nuclearnet_proxy_parser'
import { ChartSimulator } from '../virtual_robots/simulators/chart_data_simulator'
import { OverviewSimulator } from '../virtual_robots/simulators/overview_simulator'
import { SensorDataSimulator } from '../virtual_robots/simulators/sensor_data_simulator'
import { VirtualRobots } from '../virtual_robots/virtual_robots'

import { WebSocketProxyNUClearNetServer } from './nuclearnet/web_socket_proxy_nuclearnet_server'
import { WebSocketServer } from './nuclearnet/web_socket_server'

const args = minimist(process.argv.slice(2))
const withVirtualRobots = args['virtual-robots'] || false

const app = express()
const server = http.createServer(app)
const sioNetwork = sio(server, { parser: NUClearNetProxyParser } as any)

const root = `${__dirname}/../../dist`
app.use(history())
app.use(compression())
app.use(express.static(root))
app.use(favicon(`${__dirname}/../assets/favicon.ico`))

const port = process.env.PORT || 9090
server.listen(port, () => {
  // tslint:disable-next-line no-console
  console.log(`NUsight server started at http://localhost:${port}`)
})

if (withVirtualRobots) {
  const virtualRobots = VirtualRobots.of({
    fakeNetworking: true,
    numRobots: 3,
    simulators: [
      { frequency: 1, simulator: OverviewSimulator.of() },
      { frequency: 10, simulator: SensorDataSimulator.of() },
      { frequency: 10, simulator: ChartSimulator.of() },
    ],
  })
  virtualRobots.startSimulators()
}

WebSocketProxyNUClearNetServer.of(WebSocketServer.of(sioNetwork.of('/nuclearnet')), {
  fakeNetworking: withVirtualRobots,
})
