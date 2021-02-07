import stream from 'stream'

import { NbsFrame } from './nbs_frame_codecs'
import { encodeFrame } from './nbs_frame_codecs'
import { decodeFrame } from './nbs_frame_codecs'

/**
 * Encode NbsFrame objects to raw nbs frame buffers.
 *
 * Expected to be useful for writing NbsFrame objects to streams, e.g. saving them to disk with a writable file stream.
 */
export class NbsFrameEncoder extends stream.Transform {
  constructor() {
    super({
      objectMode: true,
    })
  }

  static of() {
    return new NbsFrameEncoder()
  }

  _transform(frame: NbsFrame, encoding: string, done: (err?: any, data?: any) => void) {
    this.push(encodeFrame(frame))
    done()
  }
}

/**
 * Decode raw nbs frame buffers to NbsFrame objects.
 *
 * Expected to be used in combination with NbsFrameChunker to read from a readable file stream.
 */
export class NbsFrameDecoder extends stream.Transform {
  constructor() {
    super({
      objectMode: true,
    })
  }

  static of() {
    return new NbsFrameDecoder()
  }

  _transform(buffer: Buffer, encoding: string, done: (err?: any, data?: any) => void) {
    this.push(decodeFrame(buffer))
    done()
  }
}
