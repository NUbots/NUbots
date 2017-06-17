import * as EventEmitter from 'events'
import { NUClearNetPeer } from 'nuclearnet.js'
import { NUClearNetSend } from 'nuclearnet.js'

export class FakeNUClearNetServer extends EventEmitter {
  private peers: NUClearNetPeer[]

  public constructor() {
    super()

    this.peers = []
  }

  public connect(peer: NUClearNetPeer): void {
    this.emit('nuclear_join', peer)
    this.peers.push(peer)
  }

  public disconnect(peer: NUClearNetPeer) {
    this.emit('nuclear_leave', peer)
    this.peers.splice(this.peers.indexOf(peer), 1)
  }

  public send(opts: NUClearNetSend) {
    if (typeof opts.type === 'string') {
      this.emit(opts.type, opts.payload)
    }
  }

  public static getInstance = (() => {
    let instance: FakeNUClearNetServer
    return () => {
      if (instance === undefined) {
        instance = new FakeNUClearNetServer()
      }
      return instance
    }
  })()
}
