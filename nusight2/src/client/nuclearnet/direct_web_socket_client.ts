import SocketIO from 'socket.io-client'

import * as NUClearNetProxyParser from '../../shared/nuclearnet/nuclearnet_proxy_parser'

import { WebSocketClient } from './web_socket_client'

/**
 * The thinnest wrapper possible around the Socket IO client interface. This exists to assist testing
 * WebSocketProxyNUClearNetClient by giving a concrete class which can be mocked.
 *
 * There should never be enough logic in here that it needs any testing.
 */
export class DirectWebSocketClient implements WebSocketClient {
  constructor(private socket: SocketIOClient.Socket) {}

  static of(uri: string, opts: SocketIOClient.ConnectOpts) {
    const socket = SocketIO(uri, { ...opts, parser: NUClearNetProxyParser } as any)
    return new DirectWebSocketClient(socket)
  }

  connect() {
    this.socket = this.socket.connect()
  }

  disconnect() {
    this.socket.disconnect()
  }

  on(event: string, fn: Function) {
    this.socket.on(event, fn)
  }

  off(event: string, fn?: Function) {
    this.socket.off(event, fn)
  }

  send(event: string, ...args: any[]) {
    this.socket.emit(event, ...args)
  }
}
