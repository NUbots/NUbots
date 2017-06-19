import Socket = SocketIO.Socket
import { NUClearNetPacket } from 'nuclearnet.js'
import { NUClearNet } from 'nuclearnet.js'

interface ClientOpts {
  nuclearNetwork: NUClearNet
  sioSocket: Socket
  onDisconnectCallback?: (client: Client) => void
}

export class Client {
  private nuclearNetwork: NUClearNet
  private sioSocket: Socket
  private listeners: Map<string, (event: string, packet: NUClearNetPacket) => void>
  private onDisconnectCallback?: (client: Client) => void

  public constructor(opts: ClientOpts) {
    Object.assign(this, opts)
    this.listeners = new Map()

    this.sioSocket.on('disconnect', this.onDisconnect)
    this.sioSocket.on('listen', this.onListen)
    this.sioSocket.on('unlisten', this.onUnlisten)
  }

  public static of(opts: ClientOpts) {
    return new Client(opts)
  }

  public send(event: string, ...args: any[]) {
    this.sioSocket.emit(event, ...args)
  }

  private onDisconnect = () => {
    this.unlistenAll()
    if (this.onDisconnectCallback) {
      this.onDisconnectCallback(this)
    }
  }

  private onListen = (event: string) => {
    const messageType = `NUsight<${event}>`
    // tslint:disable-next-line no-console
    console.log(`Listening for ${messageType}`)
    const cb = this.onPacket.bind(this, event)
    this.nuclearNetwork.on(messageType, cb)
    this.listeners.set(messageType, cb)
  }

  private unlistenAll() {
    for (const [messageType, cb] of this.listeners.entries()) {
      // tslint:disable-next-line no-console
      console.log(`Unlistening for ${messageType}`)
      this.nuclearNetwork.removeListener(messageType, cb)
    }
    this.listeners.clear()
  }

  private onUnlisten = (event: string) => {
    const messageType = `NUsight<${event}>`
    const cb = this.listeners.get(messageType)
    if (cb) {
      // tslint:disable-next-line no-console
      console.log(`Unlistening for ${messageType}`)
      this.nuclearNetwork.removeListener(messageType, cb)
      this.listeners.delete(messageType)
    }
  }

  private onPacket = (event: string, packet: NUClearNetPacket) => {
    this.sioSocket.emit(event, packet.payload)
  }
}
