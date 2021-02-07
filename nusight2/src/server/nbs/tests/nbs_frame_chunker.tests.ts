import { SeededRandom } from '../../../shared/base/random/seeded_random'
import { hashType } from '../../nuclearnet/fake_nuclearnet_server'
import { NbsFrameChunker } from '../nbs_frame_chunker'
import { encodeFrame } from '../nbs_frame_codecs'

describe('NbsFrameChunker', () => {
  let chunker: NbsFrameChunker
  let random: SeededRandom

  beforeEach(() => {
    chunker = NbsFrameChunker.of()
    random = SeededRandom.of('NbsFrameChunker')
  })

  function randomByte() {
    return random.integer(0, 255)
  }

  function randomBuffer(size: number) {
    const buffer = Buffer.alloc(size)
    for (let i = 0; i < size; i++) {
      buffer[i] = randomByte()
    }
    return buffer
  }

  function fakeFrameBuffer(timestampInMicroseconds: number, size: number = 8) {
    return encodeFrame({
      timestampInMicroseconds,
      hash: hashType('fake'),
      payload: randomBuffer(size),
    })
  }

  it('finds the expected frames in a contiguous stream', done => {
    const spy = jest.fn()
    const buffer1 = fakeFrameBuffer(0)
    const buffer2 = fakeFrameBuffer(1)
    const buffer3 = fakeFrameBuffer(3)

    chunker.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(buffer1)
      expect(spy).toHaveBeenCalledWith(buffer2)
      expect(spy).toHaveBeenCalledWith(buffer3)
      done()
    })

    chunker.write(buffer1)
    chunker.write(buffer2)
    chunker.write(buffer3)
    chunker.end()
  })

  it('synchronises and finds the expected frames with random leading bytes', done => {
    const spy = jest.fn()
    const buffer1 = fakeFrameBuffer(0)
    const buffer2 = fakeFrameBuffer(1)
    const buffer3 = fakeFrameBuffer(2)

    chunker.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(buffer1)
      expect(spy).toHaveBeenCalledWith(buffer2)
      expect(spy).toHaveBeenCalledWith(buffer3)
      done()
    })

    chunker.write(Buffer.alloc(32).fill(randomByte()))
    chunker.write(buffer1)
    chunker.write(buffer2)
    chunker.write(buffer3)
    chunker.end()
  })

  it('synchronises and finds the expected frames with random interleaving bytes', done => {
    const spy = jest.fn()
    const buffer1 = fakeFrameBuffer(0)
    const buffer2 = fakeFrameBuffer(1)
    const buffer3 = fakeFrameBuffer(2)

    chunker.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(buffer1)
      expect(spy).toHaveBeenCalledWith(buffer2)
      expect(spy).toHaveBeenCalledWith(buffer3)
      done()
    })

    // These random buffers are filled with a single random byte so as to not have a header
    chunker.write(Buffer.alloc(32).fill(randomByte()))
    chunker.write(buffer1)
    chunker.write(Buffer.alloc(32).fill(randomByte()))
    chunker.write(buffer2)
    chunker.write(Buffer.alloc(32).fill(randomByte()))
    chunker.write(buffer3)
    chunker.write(Buffer.alloc(32).fill(randomByte()))
    chunker.end()
  })

  it('handles packets that contain the header sequence', done => {
    const spy = jest.fn()

    let buffer1 = fakeFrameBuffer(0)
    let buffer2 = fakeFrameBuffer(1)
    let buffer3 = fakeFrameBuffer(2)

    // Wrap each buffer in another buffer that will have a header
    buffer1 = encodeFrame({
      timestampInMicroseconds: 0,
      hash: hashType('fake'),
      payload: buffer1,
    })

    buffer2 = encodeFrame({
      timestampInMicroseconds: 0,
      hash: hashType('fake'),
      payload: buffer2,
    })

    buffer3 = encodeFrame({
      timestampInMicroseconds: 0,
      hash: hashType('fake'),
      payload: buffer3,
    })

    chunker.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(buffer1)
      expect(spy).toHaveBeenCalledWith(buffer2)
      expect(spy).toHaveBeenCalledWith(buffer3)
      done()
    })

    // Randomly split up the buffer into bits
    let fullBuffer = Buffer.concat([buffer1, buffer2, buffer3])

    while (fullBuffer.length > 0) {
      // Split 100 or less to ensure we get small chunks
      const split = random.integer(1, Math.min(100, fullBuffer.length))
      chunker.write(fullBuffer.slice(0, split))
      fullBuffer = fullBuffer.slice(split)
    }

    chunker.end()
  })

  it('handles packets split over multiple buffers', done => {
    const spy = jest.fn()

    const buffer1 = fakeFrameBuffer(0, 200)
    const buffer2 = fakeFrameBuffer(1, 200)
    const buffer3 = fakeFrameBuffer(2, 200)

    chunker.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(buffer1)
      expect(spy).toHaveBeenCalledWith(buffer2)
      expect(spy).toHaveBeenCalledWith(buffer3)
      done()
    })

    // Randomly split up the buffer into bits
    let fullBuffer = Buffer.concat([buffer1, buffer2, buffer3])

    while (fullBuffer.length > 0) {
      // Split 100 or less to ensure we get small chunks
      const split = random.integer(1, Math.min(100, fullBuffer.length))
      chunker.write(fullBuffer.slice(0, split))
      fullBuffer = fullBuffer.slice(split)
    }

    chunker.end()
  })
})
