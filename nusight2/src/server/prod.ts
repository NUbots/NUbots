import compression from 'compression'
import history from 'connect-history-api-fallback'
import express from 'express'
import http from 'http'
import minimist from 'minimist'
import * as path from 'path'
import favicon from 'serve-favicon'
import { Server } from 'socket.io'

import * as NUClearNetProxyParser from '../shared/nuclearnet/nuclearnet_proxy_parser'
import { VirtualRobots } from '../virtual_robots/virtual_robots'

import { WebSocketProxyNUClearNetServer } from './nuclearnet/web_socket_proxy_nuclearnet_server'
import { WebSocketServer } from './nuclearnet/web_socket_server'

const args = minimist(process.argv.slice(2))
const withVirtualRobots = args['virtual-robots'] || false
const nuclearnetAddress = args.address || '10.1.255.255'

const app = express()
const server = http.createServer(app)
const sioNetwork = new Server(server, { parser: NUClearNetProxyParser } as any)

app.use(
  history({
    rewrites: [
      // Allows user to navigate to /storybook/ without needing to type /index.html
      { from: /\/storybook\/$/, to: 'storybook/index.html' },
    ],
  }),
)
app.use(compression())
app.use(express.static(path.join('dist', 'public')))
app.use(favicon(path.join('dist', 'public', 'favicon.ico')))

const port = process.env.PORT || 9090
server.listen(port, () => {
  // tslint:disable-next-line no-console
  console.log(`NUsight server started at http://localhost:${port}`)
})

if (withVirtualRobots) {
  const virtualRobots = VirtualRobots.of({ fakeNetworking: true, nuclearnetAddress, numRobots: 3 })
  virtualRobots.start()
}

WebSocketProxyNUClearNetServer.of(WebSocketServer.of(sioNetwork.of('/nuclearnet')), {
  fakeNetworking: withVirtualRobots,
  connectionOpts: { name: 'nusight', address: nuclearnetAddress },
})
