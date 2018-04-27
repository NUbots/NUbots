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
import * as NUClearNetProxyParser from '../shared/nuclearnet/nuclearnet_proxy_parser'
import { ChartSimulator } from '../virtual_robots/simulators/chart_data_simulator'
import { OverviewSimulator } from '../virtual_robots/simulators/overview_simulator'
import { SensorDataSimulator } from '../virtual_robots/simulators/sensor_data_simulator'
import { VirtualRobots } from '../virtual_robots/virtual_robots'

import { NBSPlayer } from './nbs/mmap_nbs_player/nbs_player'
import { NBSPacket } from './nbs/mmap_nbs_player/nbs_player'
import { DirectNUClearNetClient } from './nuclearnet/direct_nuclearnet_client'
import { FakeNUClearNetClient } from './nuclearnet/fake_nuclearnet_client'
import { WebSocketProxyNUClearNetServer } from './nuclearnet/web_socket_proxy_nuclearnet_server'
import { WebSocketServer } from './nuclearnet/web_socket_server'

const compiler = webpack(webpackConfig)

const args = minimist(process.argv.slice(2))
const withVirtualRobots = args['virtual-robots'] || false
const nbsFile = args.play

const app = express()
const server = http.createServer(app)
const sioNetwork = sio(server, { parser: NUClearNetProxyParser } as any)

// Initialize socket.io namespace immediately to catch reconnections.
WebSocketProxyNUClearNetServer.of(WebSocketServer.of(sioNetwork.of('/nuclearnet')), {
  fakeNetworking: withVirtualRobots,
})

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
  // tslint:disable-next-line no-console
  console.log(`NUsight server started at http://localhost:${port}`)
})

function init() {
  if (withVirtualRobots) {
    const virtualRobots = VirtualRobots.of({
      fakeNetworking: true,
      numRobots: 3,
      simulators: [
        { frequency: 1, simulator: OverviewSimulator.of() },
        { frequency: 60, simulator: SensorDataSimulator.of() },
        { frequency: 60, simulator: ChartSimulator.of() },
      ],
    })
    virtualRobots.startSimulators()
  }

  if (nbsFile) {
    const nuclearnetClient = withVirtualRobots ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of()
    nuclearnetClient.connect({ name: nbsFile })

    const player = NBSPlayer.of({
      file: nbsFile,
    })

    player.onPacket((packet: NBSPacket) => {
      nuclearnetClient.send({
        type: packet.hash,
        payload: packet.payload,
      })
    })

    player.onEnd(() => {
      // tslint:disable-next-line no-console
      console.log('Restarting NBS playback')
      player.restart()
      player.play()
    })

    // tslint:disable-next-line no-console
    console.log(`Playing NBS file: ${nbsFile}`)
    player.play()
  }
}

devMiddleware.waitUntilValid(init)
