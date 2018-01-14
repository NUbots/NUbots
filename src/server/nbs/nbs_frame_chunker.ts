import * as Buffers from 'buffers'
import * as stream from 'stream'

import { NBS_HEADER } from './nbs_frame_codecs'
import { PACKET_SIZE_SIZE } from './nbs_frame_codecs'

/**
 * This stream tranformer finds and emits nbs frames within a continually running binary stream. It looks for the nbs
 * header (0xe298a2) and reads just enough to know its size, and then writes nbs-packet sized buffer chunks out. In
 * other words it decodes the nbs framing protocol and emits those frames in the form of processable chunks.
 */
export class NbsFrameChunker extends stream.Transform {
  private buffers: Buffers
  private foundHeader: boolean
  private foundPacketSize: boolean

  constructor() {
    super({
      objectMode: true,
    })

    this.buffers = new Buffers()
    this.foundHeader = false
    this.foundPacketSize = false
  }

  static of(): NbsFrameChunker {
    return new NbsFrameChunker()
  }

  _transform(chunk: any, encoding: string, done: (err?: any, data?: any) => void) {
    // Buffer any received data so that we can find nbs packets within it.
    this.buffers.push(chunk)

    let frame
    // tslint:disable-next-line no-conditional-assignment
    while ((frame = this.getNextFrame(this.buffers)) !== undefined) {
      this.push(frame.buffer)
      this.buffers.splice(0, frame.index + frame.buffer.byteLength)
    }

    // If there are no headers within the data, just empty the buffer.
    // Prevents this being an unbounded buffer continually accumulating when no nbs packets are to be found.
    if (this.buffers.indexOf(NBS_HEADER) === -1) {
      this.buffers = new Buffers()
    }

    done()
  }

  /**
   * This method will find and return the next nbs frame within a buffer. It searches for the next occurrence of the
   * nbs header byte sequence, and then processes just enough to know the size of the frame and it will return that
   * slice, along with the starting index it was found at.
   */
  private getNextFrame(buffer: Buffers): { index: number, buffer: Buffer } | undefined {
    // Search for the nbs header byte sequence.
    const headerIndex = buffer.indexOf(NBS_HEADER)
    if (headerIndex >= 0) {
      // Header found, slice from that index to make the following calculations easier.
      const frame = buffer.slice(headerIndex)
      const headerSize = NBS_HEADER.byteLength
      const headerAndPacketLengthSize = headerSize + PACKET_SIZE_SIZE
      // Check that we have received enough data to read the size of the frame.
      if (frame.length >= headerAndPacketLengthSize) {
        // Read the size of the frame which exists right after the header.
        const packetSize = frame.slice(headerSize, headerSize + headerAndPacketLengthSize).readUInt32LE(0)
        // Check again that we have received enough data to the read the rest of the frame.
        if (frame.length >= headerAndPacketLengthSize + packetSize) {
          // Slice and return the entire frame, along with where we found it.
          return {
            index: headerIndex,
            buffer: frame.slice(0, headerAndPacketLengthSize + packetSize),
          }
        }
      }
    }
    // No complete frame was found.
    return undefined
  }
}
