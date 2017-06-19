import * as io from 'socket.io-client'
import Socket = SocketIOClient.Socket
import { injectable } from 'inversify'

@injectable()
export class RawSocket {
  private socket: Socket

  public static of() {
    return new RawSocket()
  }

  public on(event: string, fn: (...args: any[]) => void) {
    this.connectIfNotConnected()
    this.socket.on(event, fn)
  }

  public off(event: string, fn?: (...args: any[]) => void) {
    this.connectIfNotConnected()
    this.socket.off(event, fn)
  }

  public listen(eventName: string) {
    this.emit('listen', eventName)
  }

  public unlisten(eventName: string) {
    this.emit('unlisten', eventName)
  }

  public emit(event: string, ...args: any[]) {
    this.connectIfNotConnected()
    this.socket.emit(event, ...args)
  }

  private connectIfNotConnected() {
    if (!this.socket) {
      this.socket = io.connect()
    }
  }
}
