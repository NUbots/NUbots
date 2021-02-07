import { NUClearNetSend } from 'nuclearnet.js'

import { NUsightNetwork } from './nusight_network'
import { MessageType } from './nusight_network'
import { MessageCallback } from './nusight_network'

/**
 * A convenience helper class to be used at the component-level.
 *
 * Easily subscribe to multiple NUClearNet messages with on(MessageType) and then unsubscribe from them all with off().
 */
export class Network {
  // Store all event listener removers, so we can call them all in off().
  private offNUClearMessages: Set<() => void>

  constructor(private nusightNetwork: NUsightNetwork) {
    this.offNUClearMessages = new Set()
  }

  static of(nusightNetwork: NUsightNetwork): Network {
    return new Network(nusightNetwork)
  }

  /**
   * Subscribe to a NUClearNet message.
   *
   * @param messageType The protobuf message type that a robot would send. e.g. message.input.Sensors
   * @param cb The callback to call every time a message is received
   * @returns A unsubscriber function.
   */
  on<T>(messageType: MessageType<T>, cb: MessageCallback<T>): () => void {
    const offNUClearMessage = this.nusightNetwork.onNUClearMessage(messageType, cb)
    this.offNUClearMessages.add(offNUClearMessage)
    return () => {
      offNUClearMessage()
      this.offNUClearMessages.delete(offNUClearMessage)
    }
  }

  /**
   * Unsubscribe from all events that is currently being listened to.
   */
  off() {
    for (const offNUClearMessage of this.offNUClearMessages.values()) {
      offNUClearMessage()
    }
    this.offNUClearMessages.clear()
  }

  /**
   * Send the given message on the network
   */
  send(opts: NUClearNetSend): void {
    this.nusightNetwork.send(opts)
  }
}
