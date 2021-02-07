import bindings from 'bindings'
import { EventEmitter } from 'events'
import fs from 'fs'
import * as path from 'path'

const MMapNBSPlayer = bindings({
  bindings: 'nbs_player',
  module_root: path.join(__dirname, '..'),
} as any) as MMapNBSPlayerConstructor

export type NBSPacket = {
  timestamp: number
  hash: Buffer
  payload: Buffer
}

export class NBSPlayer {
  private readonly player: MMapNBSPlayer
  private readonly emitter: EventEmitter = new EventEmitter()

  private constructor(private file: string) {
    this.player = new MMapNBSPlayer(file, this.onNBSPacket)
  }

  static of({ file }: { file: string }) {
    if (fs.existsSync(file)) {
      return new NBSPlayer(file)
    } else {
      throw Error(`The nbs file ${file} does not exist!`)
    }
  }

  onPacket(cb: (packet: NBSPacket) => void): () => void {
    this.emitter.on('packet', cb)
    return () => {
      this.emitter.removeListener('packet', cb)
    }
  }

  onEnd(cb: () => void): () => void {
    this.emitter.on('end', cb)
    return () => {
      this.emitter.removeListener('end', cb)
    }
  }

  play() {
    this.player.play()
  }

  pause() {
    this.player.pause()
  }

  restart() {
    this.player.restart()
  }

  step(steps: number) {
    this.player.step(steps)
  }

  seek(timestamp: number) {
    this.player.seek(timestamp)
  }

  private onNBSPacket = (timestamp?: number, hash?: Buffer, payload?: Buffer) => {
    if (timestamp && hash && payload) {
      const packet: NBSPacket = { timestamp, hash, payload }
      this.emitter.emit('packet', packet)
    } else {
      this.emitter.emit('end')
    }
  }
}

type MMapNBSPlayerConstructor = new (
  file: string,
  cb: (timestamp?: number, hash?: Buffer, payload?: Buffer) => void,
) => MMapNBSPlayer

interface MMapNBSPlayer {
  play(): void

  pause(): void

  restart(): void

  step(steps: number): void

  seek(timestamp: number): void
}
