import stream from 'stream'

import { NBS_HEADER } from './nbs_frame_codecs'
import { PACKET_SIZE_SIZE } from './nbs_frame_codecs'

/**
 * This stream tranformer finds and emits nbs frames within a continually running binary stream. It looks for the nbs
 * header (0xe298a2) and reads just enough to know its size, and then writes nbs-packet sized buffer chunks out. In
 * other words it decodes the nbs framing protocol and emits those frames in the form of processable chunks.
 */
export class NbsFrameChunker extends stream.Transform {
  private buffer: Buffer
  private foundHeader: boolean
  private foundPacketSize: boolean

  constructor() {
    super({
      objectMode: true,
    })

    this.foundHeader = false
    this.foundPacketSize = false
    this.buffer = Buffer.alloc(0)
  }

  static of(): NbsFrameChunker {
    return new NbsFrameChunker()
  }

  _transform(chunk: any, encoding: string, done: (err?: any, data?: any) => void) {
    // Buffer any received data so that we can find nbs packets within it.
    this.buffer = Buffer.concat([this.buffer, chunk])

    let frame
    // tslint:disable-next-line no-conditional-assignment
    while ((frame = this.getNextFrame(this.buffer)) !== undefined) {
      this.push(frame.buffer)
      this.buffer = this.buffer.slice(frame.index + frame.buffer.byteLength)
    }

    // If there are no headers within the data, just empty the buffer.
    // Prevents this being an unbounded buffer continually accumulating when no nbs packets are to be found.
    if (this.buffer.indexOf(NBS_HEADER) === -1) {
      this.buffer = Buffer.alloc(0)
    }

    done()
  }

  /**
   * This method will find and return the next nbs frame within a buffer. It searches for the next occurrence of the
   * nbs header byte sequence, and then processes just enough to know the size of the frame and it will return that
   * slice, along with the starting index it was found at.
   */
  private getNextFrame(buffer: Buffer): { index: number; buffer: Buffer } | undefined {
    // Search for the nbs header byte sequence.
    const headerIndex = buffer.indexOf(NBS_HEADER)
    if (headerIndex >= 0 && buffer.length > headerIndex + PACKET_SIZE_SIZE) {
      // Read the size of the next packet
      const sizeStart = headerIndex + NBS_HEADER.byteLength
      const sizeEnd = sizeStart + PACKET_SIZE_SIZE
      const packetSize = buffer.slice(sizeStart, sizeEnd).readUInt32LE(0)

      // We have enough data
      if (sizeEnd + packetSize <= buffer.length) {
        return {
          index: headerIndex,
          buffer: buffer.slice(headerIndex, sizeEnd + packetSize),
        }
      }
    }
    // No complete frame was found.
    return undefined
  }
}
