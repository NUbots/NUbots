import { inject } from 'inversify'
import { injectable } from 'inversify'
import { GlobalNetwork } from './global_network'
import { MessageType } from './global_network'
import { Message } from './global_network'
import { MessageCallback } from './global_network'

@injectable()
export class Network {
  private listeners: Map<MessageType<Message>, Set<MessageCallback<Message>>>

  public constructor(@inject(GlobalNetwork) private globalNetwork: GlobalNetwork) {
    this.listeners = new Map()
  }

  public on<T extends Message>(messageType: MessageType<T>, cb: MessageCallback<T>) {
    this.globalNetwork.on(messageType, cb)

    if (!this.listeners.has(messageType)) {
      this.listeners.set(messageType, new Set())
    }
    const listeners = this.listeners.get(messageType)
    if (listeners) {
      listeners.add(cb)
    }
  }

  public off() {
    for (const [messageType, callbacks] of this.listeners.entries()) {
      for (const cb of callbacks) {
        this.globalNetwork.off(messageType, cb)
      }
    }
    this.listeners.clear()
  }
}
