import { hashType } from '../../nuclearnet/fake_nuclearnet_server'
import { encodeFrame } from '../nbs_frame_codecs'
import { NbsFrameDecoder } from '../nbs_frame_streams'
import { NbsFrameEncoder } from '../nbs_frame_streams'

describe('NbsFrameEncoder', () => {
  it('encodes frames to buffers', done => {
    const encoder = new NbsFrameEncoder()
    const frame = {
      timestampInMicroseconds: 0,
      hash: hashType('fake'),
      payload: Buffer.alloc(8).fill(12),
    }
    const buffer = encodeFrame(frame)
    const spy = jest.fn()

    encoder.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(buffer)
      done()
    })

    encoder.write(frame)
    encoder.end()
  })
})

describe('NbsFrameDecoder', () => {
  it('decodes buffers to frames', done => {
    const decoder = new NbsFrameDecoder()
    const frame = {
      timestampInMicroseconds: 0,
      hash: hashType('fake'),
      payload: Buffer.alloc(8).fill(12),
    }
    const buffer = encodeFrame(frame)
    const spy = jest.fn()

    decoder.on('data', spy).on('finish', () => {
      expect(spy).toHaveBeenCalledWith(frame)
      done()
    })

    decoder.write(buffer)
    decoder.end()
  })
})
