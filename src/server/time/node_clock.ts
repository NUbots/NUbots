import { Clock } from './clock'

export type CancelTimer = () => void

function setTimeout(cb: (...args: any[]) => void, ms: number): CancelTimer {
  const handle = global.setTimeout(cb, ms)
  return global.clearTimeout.bind(null, handle)
}

function setInterval(cb: (...args: any[]) => void, ms: number): CancelTimer {
  const handle = global.setInterval(cb, ms)
  return global.clearInterval.bind(null, handle)
}

function setImmediate(cb: (...args: any[]) => void): CancelTimer {
  const handle = global.setImmediate(cb)
  return global.clearImmediate.bind(null, handle)
}

export const NodeSystemClock: Clock = {
  now: () => Date.now(),
  setTimeout,
  setInterval,
  setImmediate,
}
