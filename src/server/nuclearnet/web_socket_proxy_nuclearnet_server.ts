import { NUClearNetSend } from 'nuclearnet.js'
import { NUClearNetOptions } from 'nuclearnet.js'
import { NUClearNetPeer } from 'nuclearnet.js'
import { NUClearNetPacket } from 'nuclearnet.js'

import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { Clock } from '../../shared/time/clock'
import { NodeSystemClock } from '../time/node_clock'

import { DirectNUClearNetClient } from './direct_nuclearnet_client'
import { FakeNUClearNetClient } from './fake_nuclearnet_client'
import { LruPriorityQueue } from './lru_priority_queue'
import { WebSocketServer } from './web_socket_server'
import { WebSocket } from './web_socket_server'

type Opts = {
  fakeNetworking: boolean
  nuclearnetAddress: string
}

/**
 * The server component of a NUClearNet proxy running over web sockets. Acts as a gateway to the NUClear network.
 * All clients currently share a single NUClearNet connection, mostly for performance reasons. Could potentially be
 * improved to have more intelligent multiplexing.
 */
export class WebSocketProxyNUClearNetServer {
  private nuclearnetAddress: string

  constructor(
    private server: WebSocketServer,
    private nuclearnetClient: NUClearNetClient,
    private address: string,
  ) {
    server.onConnection(this.onClientConnection)
    this.nuclearnetAddress = address
  }

  static of(
    server: WebSocketServer,
    { fakeNetworking, nuclearnetAddress }: Opts,
  ): WebSocketProxyNUClearNetServer {
    const nuclearnetClient: NUClearNetClient = fakeNetworking
      ? FakeNUClearNetClient.of()
      : DirectNUClearNetClient.of()
    return new WebSocketProxyNUClearNetServer(server, nuclearnetClient, nuclearnetAddress)
  }

  private onClientConnection = (socket: WebSocket) => {
    WebSocketServerClient.of(this.nuclearnetClient, socket, this.nuclearnetAddress)
  }
}

class WebSocketServerClient {
  private connected: boolean
  private offJoin: () => void
  private offLeave: () => void
  private offListenMap: Map<string, () => void>
  private nuclearnetAddress: string

  constructor(
    private nuclearnetClient: NUClearNetClient,
    private socket: WebSocket,
    private processor: PacketProcessor,
    private address: string,
  ) {
    this.connected = false
    this.offJoin = this.nuclearnetClient.onJoin(this.onJoin)
    this.offLeave = this.nuclearnetClient.onLeave(this.onLeave)
    this.offListenMap = new Map()
    this.nuclearnetAddress = this.address

    this.socket.on('packet', this.onClientPacket)
    this.socket.on('listen', this.onListen)
    this.socket.on('unlisten', this.onUnlisten)
    this.socket.on('nuclear_connect', this.onConnect)
    this.socket.on('disconnect', this.onDisconnect)
  }

  static of(nuclearnetClient: NUClearNetClient, socket: WebSocket, nuclearnetAddress: string) {
    return new WebSocketServerClient(
      nuclearnetClient,
      socket,
      PacketProcessor.of(socket),
      nuclearnetAddress,
    )
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
      options.address = this.nuclearnetAddress
      const disconnect = this.nuclearnetClient.connect(options)
      this.connected = true
      this.socket.on('nuclear_disconnect', () => {
        disconnect()
        this.connected = false
      })
    }
  }

  private onListen = (event: string, requestToken: string) => {
    const off = this.nuclearnetClient.on(event, this.onServerPacket.bind(this, event))
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

  private onServerPacket = (event: string, packet: NUClearNetPacket) => {
    this.processor.onPacket(event, packet)
  }

  private onClientPacket = (options: NUClearNetSend) => {
    this.nuclearnetClient.send(options)
  }
}

class PacketProcessor {
  private outgoingPackets: number = 0

  // The maximum number of packets to send before receiving acknowledgements.
  private outgoingLimit: number

  // The number of seconds before giving up on an acknowledge
  private timeout: number

  constructor(
    private socket: WebSocket,
    private clock: Clock,
    private queue: LruPriorityQueue<string, { event: string; packet: NUClearNetPacket }>,
    { outgoingLimit, timeout }: { outgoingLimit: number; timeout: number },
  ) {
    this.outgoingLimit = outgoingLimit
    this.timeout = timeout
    this.queue = queue
  }

  static of(socket: WebSocket) {
    return new PacketProcessor(
      socket,
      NodeSystemClock,
      new LruPriorityQueue({ capacityPerKey: 2 }),
      { outgoingLimit: 10, timeout: 5 },
    )
  }

  onPacket(event: string, packet: NUClearNetPacket) {
    if (packet.reliable) {
      // Always send reliable packets
      this.socket.send(event, packet)
    } else {
      // Throttle unreliable packets so that we do not overwhelm the client with traffic.
      const key = `${event}:${packet.peer.name}:${packet.peer.address}:${packet.peer.port}`
      this.queue.add(key, { event, packet })
      this.maybeSendNextPacket()
    }
  }

  private maybeSendNextPacket() {
    if (this.outgoingPackets < this.outgoingLimit) {
      const next = this.queue.pop()
      if (next) {
        const { event, packet } = next
        let isDone = false
        const done = () => {
          if (!isDone) {
            this.outgoingPackets--
            isDone = true
            this.maybeSendNextPacket()
          }
        }
        this.outgoingPackets++
        this.socket.volatileSend(event, packet, done)
        this.clock.setTimeout(done, this.timeout)
      }
    }
  }
}
