import * as EventEmitter from 'events'
import { injectable } from 'inversify'
import { NUClearNetPeer } from 'nuclearnet.js'
import { NUClearNetSend } from 'nuclearnet.js'

@injectable()
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

  public send(peer: NUClearNetPeer, opts: NUClearNetSend) {
    if (typeof opts.type === 'string') {
      const packet = {
        peer,
        payload: opts.payload,
      }
      // TODO (Annable): Support opts.target
      this.emit(opts.type, packet)
    }
  }
}
