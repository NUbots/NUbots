import { NUClearNet } from 'nuclearnet.js'
import { NUClearNetOptions } from 'nuclearnet.js'
import { NUClearNetSend } from 'nuclearnet.js'

import { NUClearPacketListener } from '../../shared/nuclearnet/nuclearnet_client'
import { NUClearEventListener } from '../../shared/nuclearnet/nuclearnet_client'
import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'

/**
 * A thin adapter around the real NUClearNet which implements the NUClearNetClient interface.
 */
export class DirectNUClearNetClient implements NUClearNetClient {
  constructor(private nuclearNetwork: NUClearNet) {}

  static of(): DirectNUClearNetClient {
    const nuclearNetwork = new NUClearNet()
    return new DirectNUClearNetClient(nuclearNetwork)
  }

  connect(options: NUClearNetOptions): () => void {
    this.nuclearNetwork.connect(options)
    return () => this.nuclearNetwork.disconnect()
  }

  onJoin(cb: NUClearEventListener): () => void {
    this.nuclearNetwork.on('nuclear_join', cb)
    return () => this.nuclearNetwork.removeListener('nuclear_join', cb)
  }

  onLeave(cb: NUClearEventListener): () => void {
    this.nuclearNetwork.on('nuclear_leave', cb)
    return () => this.nuclearNetwork.removeListener('nuclear_leave', cb)
  }

  on(event: string, cb: NUClearPacketListener): () => void {
    this.nuclearNetwork.on(event, cb)
    return () => this.nuclearNetwork.removeListener(event, cb)
  }

  onPacket(cb: NUClearPacketListener): () => void {
    return this.on('nuclear_packet', cb)
  }

  send(options: NUClearNetSend): void {
    this.nuclearNetwork.send(options)
  }
}
