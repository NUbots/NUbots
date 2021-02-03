import { ReadStream } from 'fs'
import fs from 'fs'
import stream from 'stream'
import { PassThrough } from 'stream'
import { createGunzip } from 'zlib'

import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { Clock } from '../../shared/time/clock'
import { NodeSystemClock } from '../time/node_clock'

import { NbsFrameChunker } from './nbs_frame_chunker'
import { NbsFrame } from './nbs_frame_codecs'
import { NbsFrameDecoder } from './nbs_frame_streams'

/**
 * This writable stream takes NbsFrame objects and put them on the NUClearNet network using the given NUClearNetClient.
 *
 * This is a similar concept to a virtual robot, but instead of simulating, it has been designed just to "playback" the
 * given frames as if they were real.
 *
 * It also takes into account the timestampInMicroseconds property which exists on every NbsFrame and ensures that the
 * playback speed is consistent with original values recorded. Taking the first frame it receives as a reference point,
 * all subsequent frames will only be sent to the network at the appropriate time that is relative to the previous
 * frames.
 */
export class NbsNUClearPlayback extends stream.Writable {
  private firstFrameTimestamp?: number
  private firstLocalTimestamp?: number

  constructor(private nuclearnetClient: NUClearNetClient, private clock: Clock) {
    super({
      objectMode: true,
    })
  }

  static of(nuclearnetClient: NUClearNetClient) {
    return new NbsNUClearPlayback(nuclearnetClient, NodeSystemClock)
  }

  /** Convenience method for directly streaming a file to the network. */
  static fromFile(filename: string, nuclearnetClient: NUClearNetClient) {
    const playback = NbsNUClearPlayback.of(nuclearnetClient)
    // This high water mark is to ensure that we don't spend much time splitting/joining buffers
    // 32MB is large enough that there will be few packets that must be split regularly
    const rawStream = fs.createReadStream(filename, { highWaterMark: 1024 * 1024 * 32 })
    const isGzipped = filename.endsWith('.nbz') || filename.endsWith('.nbs.gz')
    const decompress = isGzipped ? createGunzip() : new PassThrough()
    rawStream
      .pipe(decompress)
      .pipe(new NbsFrameChunker())
      .pipe(new NbsFrameDecoder())
      .pipe(playback)
    return playback
  }

  static fromRawStream(rawStream: ReadStream, nuclearnetClient: NUClearNetClient) {
    const playback = NbsNUClearPlayback.of(nuclearnetClient)
    rawStream.pipe(new NbsFrameChunker()).pipe(new NbsFrameDecoder()).pipe(playback)
    return playback
  }

  _write(frame: NbsFrame, encoding: string, done: Function) {
    const now = this.clock.performanceNow()
    if (this.firstFrameTimestamp === undefined || this.firstLocalTimestamp === undefined) {
      // This is the first frame we received, use this as reference point.
      this.firstFrameTimestamp = frame.timestampInMicroseconds
      this.firstLocalTimestamp = now
    }

    // Calculate the relative time offset this frame has from the first frame we ever got. Convert to seconds.
    const frameTimeOffset = (frame.timestampInMicroseconds - this.firstFrameTimestamp) * 1e-6
    // Convert that offset to our own time (already seconds).
    const localTimeOffset = this.firstLocalTimestamp + frameTimeOffset
    // Calculate how long that is from right now, ensure we keep it above 0.
    const timeout = Math.max(0, localTimeOffset - now)

    // Schedule the frame to be sent over the network at the precisely the right time in the future.
    this.clock.setTimeout(() => {
      this.nuclearnetClient.send({
        type: frame.hash,
        payload: frame.payload,
      })
      /*
       * Only signal we're done with the frame after we send it to the network, this should ensure we put back-pressure
       * upstream. e.g. if this was coming from a very large binary file recording, that upstream file reader would
       * only read just enough to continuously feed this stream, rather than reading the entire file.
       */
      done()
    }, timeout)
  }
}
