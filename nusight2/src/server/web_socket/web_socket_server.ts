import SocketIO from "socket.io";

import { ClientConnection } from "./client_connection";

/**
 * The thinnest wrapper possible around the Socket IO server interface. This exists to assist testing
 * WebSocketProxyNUClearNetServer by giving a concrete class which can be mocked.
 *
 * There should never be enough logic in here that it needs any testing.
 */
export class WebSocketServer {
  constructor(private sioServer: SocketIO.Server | SocketIO.Namespace) {}

  static of(server: SocketIO.Server | SocketIO.Namespace) {
    return new WebSocketServer(server);
  }

  /** Register a callback to be called when a client connects to the server. */
  onConnection(cb: (socket: WebSocket) => void): () => void {
    const listener = (socket: SocketIO.Socket) => {
      const webSocket = WebSocket.of(socket);
      cb(webSocket);
    };
    this.sioServer.on("connection", listener);
    // The cast is necessary as the SocketIO typescript definitions do not include the removeListener method.
    return () => (this.sioServer as any).removeListener("connection", listener);
  }
}

/**
 * A thin wrapper around the Socket IO socket interface, to assist testing.
 * There should never be enough logic in here that it needs any testing.
 */
export class WebSocket implements ClientConnection {
  constructor(private sioSocket: SocketIO.Socket) {}

  static of(socket: SocketIO.Socket) {
    return new WebSocket(socket);
  }

  onDisconnect(cb: (reason: string) => void) {
    return this.on("disconnect", cb);
  }

  on(event: string, cb: (...args: any[]) => void): () => void {
    this.sioSocket.on(event, cb);
    return () => this.sioSocket.off(event, cb);
  }

  send(event: string, ...args: any[]) {
    this.sioSocket.emit(event, ...args);
  }
}
