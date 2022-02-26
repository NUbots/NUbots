import Emitter from 'component-emitter'
import { SocketOptions } from 'socket.io-client'

import WebSocketWorker from './webworker_web_socket_client.worker'
import { WebSocketClient } from './web_socket_client'

export class WebWorkerWebSocketClient extends Emitter implements WebSocketClient {
  constructor(private worker: Worker) {
    super()
    worker.addEventListener('message', this.handleMessage)
  }

  static of(uri: string, opts: SocketOptions) {
    const worker = new WebSocketWorker()
    worker.postMessage({
      command: 'construct',
      uri,
      opts,
    })
    return new WebWorkerWebSocketClient(worker)
  }

  private handleMessage = (e: MessageEvent) => {
    const args = e.data.args.map((v: any) => {
      // If this is a webworker callback we need to remap it into a function
      if (typeof v._webworkerCallback === 'number') {
        return (...args: any[]) => {
          this.worker.postMessage({
            command: 'callback',
            id: v._webworkerCallback,
            args,
          })
        }
      } else {
        return v
      }
    })

    this.emit(e.data.event, ...args)
  }

  connect() {
    this.worker.postMessage({
      command: 'connect',
    })
  }

  disconnect() {
    this.worker.postMessage({
      command: 'disconnect',
    })
  }

  on(event: string, fn: Function) {
    this.worker.postMessage({
      command: 'on',
      event,
    })
    return super.on(event, fn)
  }

  off(event: string, fn?: Function) {
    this.worker.postMessage({
      command: 'off',
      event,
    })
    return super.off(event, fn)
  }

  send(event: string, ...args: any[]) {
    this.worker.postMessage({
      command: 'send',
      event,
      args,
    })
  }
}
