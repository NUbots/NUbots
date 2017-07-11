import { NUClearNetOptions } from 'nuclearnet.js'
import { NUClearNetPeer } from 'nuclearnet.js'
import { NUClearNetPacket } from 'nuclearnet.js'
import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { DirectNUClearNetClient } from './direct_nuclearnet_client'
import { FakeNUClearNetClient } from './fake_nuclearnet_client'
import { WebSocketServer } from './web_socket_server'
import { WebSocket } from './web_socket_server'

type Opts = {
  fakeNetworking: boolean
}

/**
 * The server component of a NUClearNet proxy running over web sockets. Acts as a gateway to the NUClear network.
 * All clients currently share a single NUClearNet connection, mostly for performance reasons. Could potentially be
 * improved to have more intelligent multiplexing.
 */
export class WebSocketProxyNUClearNetServer {
  public constructor(private server: WebSocketServer, private nuclearnetClient: NUClearNetClient) {
    server.onConnection(this.onClientConnection)
  }

  public static of(server: WebSocketServer, { fakeNetworking }: Opts): WebSocketProxyNUClearNetServer {
    const nuclearnetClient: NUClearNetClient = fakeNetworking ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of()
    return new WebSocketProxyNUClearNetServer(server, nuclearnetClient)
  }

  private onClientConnection = (socket: WebSocket) => {
    WebSocketServerClient.of(this.nuclearnetClient, socket)
  }
}

class WebSocketServerClient {
  private connected: boolean
  private offJoin: () => void
  private offLeave: () => void
  private offListenMap: Map<string, () => void>

  public constructor(private nuclearnetClient: NUClearNetClient, private socket: WebSocket) {
    this.connected = false
    this.offJoin = this.nuclearnetClient.onJoin(this.onJoin)
    this.offLeave = this.nuclearnetClient.onLeave(this.onLeave)
    this.offListenMap = new Map()

    this.socket.on('listen', this.onListen)
    this.socket.on('unlisten', this.onUnlisten)
    this.socket.on('nuclear_connect', this.onConnect)
    this.socket.on('disconnect', this.onDisconnect)
  }

  public static of(nuclearNetClient: NUClearNetClient, socket: WebSocket) {
    return new WebSocketServerClient(nuclearNetClient, socket)
  }

  private onJoin = (peer: NUClearNetPeer) => {
    this.socket.send('nuclear_join', peer)
  }

  private onLeave = (peer: NUClearNetPeer) => {
    this.socket.send('nuclear_leave', peer)
  }

  private onConnect = (options: NUClearNetOptions) => {
    // This could be improved.
    // Currently only listens to the first connection request and ignores subsequent requests.
    // Smarter multiplexing of the shared connection would be ideal.
    if (!this.connected) {
      const disconnect = this.nuclearnetClient.connect(options)
      this.connected = true
      this.socket.on('nuclear_disconnect', () => {
        disconnect()
        this.connected = false
      })
    }
  }

  private onListen = (event: string, requestToken: string) => {
    const off = this.nuclearnetClient.on(event, this.onPacket.bind(this, event))
    this.offListenMap.set(requestToken, off)
  }

  private onUnlisten = (requestToken: string) => {
    const off = this.offListenMap.get(requestToken)
    if (off) {
      off()
    }
  }

  private onDisconnect = () => {
    this.offJoin()
    this.offLeave()
  }

  private onPacket = (event: string, packet: NUClearNetPacket) => {
    this.socket.send(event, packet)
  }
}
