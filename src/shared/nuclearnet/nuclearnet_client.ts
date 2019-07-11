import { NUClearNetPacket } from 'nuclearnet.js'
import { NUClearNetPeer } from 'nuclearnet.js'
import { NUClearNetOptions } from 'nuclearnet.js'
import { NUClearNetSend } from 'nuclearnet.js'

export type NUClearPacketListener = (packet: NUClearNetPacket) => void

export type NUClearEventListener = (peer: NUClearNetPeer) => void

export interface NUClearNetClient {
  connect(options: NUClearNetOptions): () => void

  onJoin(cb: NUClearEventListener): () => void

  onLeave(cb: NUClearEventListener): () => void

  on(event: string, cb: NUClearPacketListener): () => void

  onPacket(cb: NUClearPacketListener): () => void

  send(options: NUClearNetSend): void
}
