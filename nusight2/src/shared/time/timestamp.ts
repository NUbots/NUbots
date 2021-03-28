import { google } from '../messages'
import Timestamp = google.protobuf.ITimestamp

export function toSeconds(timestamp?: Timestamp | null): number {
  if (!timestamp) {
    timestamp = { seconds: 0, nanos: 0 }
  }
  const seconds: number = Number(timestamp.seconds)
  const nanos: number = timestamp.nanos! || 0
  return seconds + nanos * 1e-9
}

export function toTimestamp(time: number) {
  return {
    seconds: Math.floor(time),
    nanos: (time - Math.floor(time)) * 1e9,
  }
}
