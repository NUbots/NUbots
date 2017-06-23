import { message } from '../../shared/proto/messages'
import { MessageTypePath } from './message_type_names'
import { RawSocket } from './raw_socket'
import Sensors = message.input.Sensors

export class GlobalNetwork {
  private listeners: Map<MessageType<Message>, Set<MessageCallback<Message>>>
  private packetListeners: Map<string, (messageType: MessageType<Message>, buffer: ArrayBuffer) => void>

  public constructor(private socket: RawSocket,
                     private messageTypePath: MessageTypePath) {
    this.listeners = new Map()
    this.packetListeners = new Map()
  }

  public static of() {
    const messageTypePath = MessageTypePath.of()
    const socket = RawSocket.of()
    return new GlobalNetwork(socket, messageTypePath)
  }

  public on<T extends Message>(messageType: MessageType<T>, cb: MessageCallback<T>) {
    if (!this.listeners.has(messageType)) {
      this.listeners.set(messageType, new Set())
    }
    const listeners = this.listeners.get(messageType)
    if (listeners) {
      const messageTypeName = this.messageTypePath.getPath(messageType)
      listeners.add(cb)
      if (listeners.size === 1) {
        this.socket.listen(messageTypeName)
        const listener = this.onPacket.bind(this, messageType)
        this.socket.on(messageTypeName, listener)
        this.packetListeners.set(messageTypeName, listener)
      }
    }
  }

  public off<T extends Message>(messageType: MessageType<T>, cb: MessageCallback<T>) {
    const listeners = this.listeners.get(messageType)
    if (listeners) {
      const messageTypeName = this.messageTypePath.getPath(messageType)
      listeners.delete(cb)
      const packetListener = this.packetListeners.get(messageTypeName)
      if (packetListener) {
        this.socket.off(messageTypeName, packetListener)
        if (listeners.size === 0) {
          this.socket.unlisten(messageTypeName)
          this.packetListeners.delete(messageTypeName)
        }
      }
    }
  }

  private onPacket(messageType: MessageType<Message>, buffer: ArrayBuffer) {
    const message = messageType.decode(new Uint8Array(buffer).slice(9))
    const listeners = this.listeners.get(messageType)
    if (listeners) {
      for (const listener of listeners.values()) {
        listener(message)
      }
    }
  }
}

export interface Message {
}

export interface MessageType<T extends Message> {
  new(...args: any[]): T
  decode(...args: any[]): T
}

export type MessageCallback<T extends Message> = (message: T) => void
