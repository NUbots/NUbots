import { Clock } from './clock'

export type CancelTimer = () => void

function setTimeout(cb: (...args: any[]) => void, seconds: number): CancelTimer {
  const handle = global.setTimeout(cb, seconds * 1e3)
  return global.clearTimeout.bind(null, handle)
}

function setInterval(cb: (...args: any[]) => void, seconds: number): CancelTimer {
  const handle = global.setInterval(cb, seconds * 1e3)
  return global.clearInterval.bind(null, handle)
}

function setImmediate(cb: (...args: any[]) => void): CancelTimer {
  const handle = global.setImmediate(cb)
  return global.clearImmediate.bind(null, handle)
}

function performanceNow() {
  const t = process.hrtime()
  return t[0] + t[1] * 1e-9
}


export const NodeSystemClock: Clock = {
  now: () => Date.now() * 1e-3,
  performanceNow,
  setTimeout,
  setInterval,
  setImmediate,
}
