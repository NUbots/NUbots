import { NUClearNet } from 'nuclearnet.js'
import { NUClearNetPacket } from 'nuclearnet.js'
import { NUClearNetOptions } from 'nuclearnet.js'
import { NUClearNetSend } from 'nuclearnet.js'
import { NUClearNetPeer } from 'nuclearnet.js'
import { FakeNUClearNetServer } from './fake_nuclearnet_server'

type PacketListener = (packet: NUClearNetPacket) => void
type EventListener = (peer: NUClearNetPeer) => void
type Listener = EventListener | PacketListener
type EventMessage = 'nuclear_join' | 'nuclear_leave'

export class FakeNUClearNet implements NUClearNet {
  private server: FakeNUClearNetServer
  private peer: NUClearNetPeer
  private listeners: Map<Listener, Listener>
  private connected: boolean

  public constructor(server: FakeNUClearNetServer) {
    this.server = server
    this.listeners = new Map()
    this.connected = false
  }

  public static of() {
    const server = FakeNUClearNetServer.getInstance()
    return new FakeNUClearNet(server)
  }

  public on(event: EventMessage, callback: EventListener): this
  public on(event: string, callback: PacketListener): this
  public on(event: string, callback: Listener): this {
    const listener = (...args: any[]) => {
      if (this.connected) {
        callback.apply(null, args)
      }
    }
    this.listeners.set(callback, listener)
    this.server.on(event, listener)
    return this
  }

  public removeListener(event: string, callback: Listener): this {
    const listener = this.listeners.get(callback)
    if (listener) {
      this.server.removeListener(event, listener)
      this.listeners.delete(callback)
    }
    return this
  }

  public connect(options: NUClearNetOptions): void {
    this.peer = {
      name: options.name,
      address: '127.0.0.1',
      port: options.port || 7447,
    }
    this.server.connect(this.peer)
    this.connected = true
  }

  public disconnect(): void {
    if (this.peer === undefined) {
      throw new Error('Cannot disconnect without first connecting')
    }
    this.connected = false
    this.server.disconnect(this.peer)
  }

  public send(options: NUClearNetSend): void {
    this.server.send(options)
  }
}
