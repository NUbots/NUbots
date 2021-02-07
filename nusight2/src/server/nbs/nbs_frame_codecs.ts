import assert from 'assert'
import binary from 'binary'
import Long from 'long'
import { NUClearNetPacket } from 'nuclearnet.js'

export type NbsFrame = {
  // Omitted redundant header information.
  // header: Buffer
  // size: number
  timestampInMicroseconds: number
  hash: Buffer
  payload: Buffer
}

// NBS frame format:
// 3 Bytes - NUClear radiation symbol header, useful for synchronisation when attaching to an existing stream.
// 4 Bytes - The remaining packet length i.e. 16 bytes + N payload bytes
// 8 Bytes - 64bit timestamp in microseconds. Note: this is not necessarily a unix timestamp.
// 8 Bytes - 64bit bit hash of the message type.
// N bytes - The binary packet payload.

export const NBS_HEADER = Buffer.from([0xe2, 0x98, 0xa2]) // NUClear radiation symbol.
export const PACKET_SIZE_SIZE = 4
export const TIMESTAMP_SIZE = 8
export const HASH_SIZE = 8

export function encodeFrame(frame: NbsFrame): Buffer {
  assert(
    frame.hash.byteLength === HASH_SIZE,
    `Expected hash buffer size of ${HASH_SIZE} but received ${frame.hash.byteLength}`,
  )

  const size = TIMESTAMP_SIZE + HASH_SIZE + frame.payload.byteLength
  const sizeBuffer = Buffer.alloc(PACKET_SIZE_SIZE)
  sizeBuffer.writeUInt32LE(size, 0)

  const timeLong = Long.fromNumber(frame.timestampInMicroseconds)
  const timestampBuffer = Buffer.alloc(TIMESTAMP_SIZE)
  timestampBuffer.writeUInt32LE(timeLong.getLowBitsUnsigned(), 0)
  timestampBuffer.writeUInt32LE(timeLong.getHighBitsUnsigned(), 4)

  return Buffer.concat([NBS_HEADER, sizeBuffer, timestampBuffer, frame.hash, frame.payload])
}

export function decodeFrame(buffer: Buffer): NbsFrame {
  const values = binary
    .parse(buffer)
    .buffer('header', NBS_HEADER.byteLength)
    .word32lu('size')
    .word64le('timestampInMicroseconds')
    .buffer('hash', HASH_SIZE)
    .tap(function (vars) {
      this.buffer('payload', vars.size)
    }).vars

  return {
    timestampInMicroseconds: values.timestampInMicroseconds,
    hash: values.hash,
    payload: values.payload,
  }
}

export function packetToFrame(packet: NUClearNetPacket, timestamp: number): NbsFrame {
  return {
    timestampInMicroseconds: timestamp * 1e6,
    hash: packet.hash,
    payload: packet.payload,
  }
}
