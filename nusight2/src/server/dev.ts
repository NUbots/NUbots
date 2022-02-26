import compression from 'compression'
import history from 'connect-history-api-fallback'
import express from 'express'
import http from 'http'
import minimist from 'minimist'
import favicon from 'serve-favicon'
import { Server } from 'socket.io'
import webpack from 'webpack'
import webpackDevMiddleware from 'webpack-dev-middleware'
import * as path from 'path'

import { getClientConfig } from '../../webpack.config'
import faviconPath from '../assets/favicon.ico'
import * as NUClearNetProxyParser from '../shared/nuclearnet/nuclearnet_proxy_parser'
import { VirtualRobots } from '../virtual_robots/virtual_robots'

import { NBSPlayer } from './nbs/mmap_nbs_player/nbs_player'
import { NBSPacket } from './nbs/mmap_nbs_player/nbs_player'
import { DirectNUClearNetClient } from './nuclearnet/direct_nuclearnet_client'
import { FakeNUClearNetClient } from './nuclearnet/fake_nuclearnet_client'
import { WebSocketProxyNUClearNetServer } from './nuclearnet/web_socket_proxy_nuclearnet_server'
import { WebSocketServer } from './nuclearnet/web_socket_server'

const args = minimist(process.argv.slice(2))
const compiler = webpack(
  getClientConfig({
    mode: 'development',
    context: args.context || undefined,
    transpileOnly: args.t || args.transpileOnly || false,
    rootDir: path.join(__dirname, '..'),
  }),
)

const withVirtualRobots = args['virtual-robots'] || false
const nbsFile = args.play
const nuclearnetAddress = args.address || '10.1.255.255'

const app = express()
const server = http.createServer(app)
const sioNetwork = new Server(server, { parser: NUClearNetProxyParser })

// Initialize socket.io namespace immediately to catch reconnections.
WebSocketProxyNUClearNetServer.of(WebSocketServer.of(sioNetwork.of('/nuclearnet')), {
  fakeNetworking: withVirtualRobots,
  connectionOpts: { name: 'nusight', address: nuclearnetAddress },
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
app.use(favicon(faviconPath))

const port = process.env.PORT || 3000
server.listen(port, () => {
  // tslint:disable-next-line no-console
  console.log(`NUsight server started at http://localhost:${port}`)
})

function init() {
  if (withVirtualRobots) {
    const virtualRobots = VirtualRobots.of({
      fakeNetworking: true,
      nuclearnetAddress,
      numRobots: 3,
    })
    virtualRobots.start()
  }

  if (nbsFile) {
    const nuclearnetClient = withVirtualRobots
      ? FakeNUClearNetClient.of()
      : DirectNUClearNetClient.of()
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
