import * as SocketIO from 'socket.io'

/**
 * The thinnest wrapper possible around the Socket IO server interface. This exists to assist testing
 * WebSocketProxyNUClearNetServer by giving a concrete class which can be mocked.
 *
 * There should never be enough logic in here that it needs any testing.
 */
export class WebSocketServer {
  constructor(private sioServer: SocketIO.Server | SocketIO.Namespace) {
  }

  static of(server: SocketIO.Server | SocketIO.Namespace) {
    return new WebSocketServer(server)
  }

  onConnection(cb: (socket: WebSocket) => void) {
    this.sioServer.on('connection', (socket: SocketIO.Socket) => {
      const webSocket = WebSocket.of(socket)
      cb(webSocket)
    })
  }
}

export class WebSocket {
  constructor(private sioSocket: SocketIO.Socket) {
  }

  static of(socket: SocketIO.Socket) {
    return new WebSocket(socket)
  }

  on(event: string, cb: (...args: any[]) => void) {
    this.sioSocket.on(event, cb)
  }

  send(event: string, ...args: any[]) {
    this.sioSocket.emit(event, ...args)
  }
}
