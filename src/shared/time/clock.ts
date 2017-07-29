export interface Clock {
  now(): number
  performanceNow(): number
  setTimeout(cb: (...args: any[]) => void, seconds: number): CancelTimer
  setInterval(cb: (...args: any[]) => void, seconds: number): CancelTimer
  setImmediate(cb: (...args: any[]) => void): CancelTimer
}

export type CancelTimer = () => void

