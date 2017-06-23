import * as compression from 'compression'
import * as history from 'connect-history-api-fallback'
import * as express from 'express'
import * as http from 'http'
import * as minimist from 'minimist'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'
import * as webpack from 'webpack'
import * as webpackDevMiddleware from 'webpack-dev-middleware'
import * as webpackHotMiddleware from 'webpack-hot-middleware'
import webpackConfig from '../../webpack.config'
import { RobotSimulator } from '../simulators/robot_simulator'
import { SimulatorStatus } from '../simulators/robot_simulator'
import { SensorDataSimulator } from '../simulators/sensor_data_simulator'
import { NUSightServer } from './app/server'
import CloseTo = Chai.CloseTo

const compiler = webpack(webpackConfig)

const args = minimist(process.argv.slice(2))
const withSimulators = args['with-simulators'] || false

const app = express()
const server = http.createServer(app)
const sioNetwork = sio(server)

const devMiddleware = webpackDevMiddleware(compiler, {
  publicPath: '/',
  index: 'index.html',
  stats: {
    colors: true,
  },
})

app.use(compression())
// We need to wrap the fallback history API with two instances of the dev middleware to handle the initial raw request
// and the following rewritten request.
// Refer to: https://github.com/webpack/webpack-dev-middleware/pull/44#issuecomment-170462282
app.use(devMiddleware)
app.use(history())
app.use(devMiddleware)
app.use(webpackHotMiddleware(compiler))
app.use(favicon(`${__dirname}/../assets/favicon.ico`))

const port = process.env.PORT || 3000
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
  SimulatorStatus.of(robotSimulator).statusEvery(60)
}

NUSightServer.of(withSimulators, sioNetwork).connect()
