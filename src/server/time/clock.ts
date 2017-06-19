export const ClockType = Symbol('Clock')

export interface Clock {
  now(): number
  setTimeout(cb: (...args: any[]) => void, ms: number): () => void
  setInterval(cb: (...args: any[]) => void, ms: number): () => void
  setImmediate(cb: (...args: any[]) => void): () => void
}
