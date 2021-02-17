import { NUClearNetPacket } from 'nuclearnet.js'

import { hashType } from '../../nuclearnet/fake_nuclearnet_server'
import { encodeFrame } from '../nbs_frame_codecs'
import { decodeFrame } from '../nbs_frame_codecs'
import { packetToFrame } from '../nbs_frame_codecs'

describe('NbsFrameCodecs', () => {
  describe('encoding', () => {
    it('encodes frames', () => {
      const hash = hashType('message.input.sensors')
      const timestamp = 1500379664696000
      const payload = Buffer.alloc(8).fill(0x12)
      const buffer = encodeFrame({ timestampInMicroseconds: timestamp, hash, payload })
      expect(buffer.toString('hex')).toEqual(
        'e298a218000000c042f15c9654050010abef8b5398f0d41212121212121212',
      )
    })

    it('encodes timestamps as unsigned integers', () => {
      const hash = hashType('message.input.sensors')
      const timestamp = -1 >>> 0 // Take any negative value and convert it to an unsigned integer.
      const payload = Buffer.alloc(8).fill(0x12)
      const buffer = encodeFrame({ timestampInMicroseconds: timestamp, hash, payload })
      expect(buffer.toString('hex')).toEqual(
        'e298a218000000ffffffff0000000010abef8b5398f0d41212121212121212',
      )
    })

    it('errors if you supply an invalid hash size', () => {
      const hash = Buffer.alloc(12)
      const timestamp = 1500379664696000
      const payload = Buffer.alloc(8).fill(0x12)
      const actual = () => encodeFrame({ timestampInMicroseconds: timestamp, hash, payload })
      expect(actual).toThrowError(/Expected hash buffer size/)
    })

    it('encodes NUClearNet packets', () => {
      const packet: NUClearNetPacket = {
        peer: { name: 'Bob', address: '127.0.0.1', port: 1234 },
        hash: hashType('message.input.sensors'),
        payload: Buffer.alloc(8).fill(0x12),
        reliable: false,
      }

      expect(packetToFrame(packet, 2)).toEqual({
        timestampInMicroseconds: 2000000,
        hash: packet.hash,
        payload: packet.payload,
      })
    })
  })

  describe('decoding', () => {
    it('decodes frames', () => {
      const buffer = Buffer.from(
        'e298a218000000c042f15c9654050010abef8b5398f0d41212121212121212',
        'hex',
      )
      const frame = decodeFrame(buffer)
      expect(frame).toEqual({
        timestampInMicroseconds: 1500379664696000,
        hash: hashType('message.input.sensors'),
        payload: Buffer.alloc(8).fill(0x12),
      })
    })
  })

  describe('roundtrip', () => {
    it('decode than encode should equal original', () => {
      const buffer = Buffer.from(
        'e298a218000000c042f15c9654050010abef8b5398f0d41212121212121212',
        'hex',
      )
      expect(encodeFrame(decodeFrame(buffer))).toEqual(buffer)
    })

    it('encode than decode should equal original', () => {
      const frame = {
        hash: hashType('message.input.sensors'),
        timestampInMicroseconds: 1500379664696000,
        payload: Buffer.alloc(8).fill(0x12),
      }
      expect(decodeFrame(encodeFrame(frame))).toEqual(frame)
    })
  })
})
