export interface Clock {
  now(): number
  performanceNow(): number
  setTimeout(cb: (...args: any[]) => void, seconds: number): () => void
  setInterval(cb: (...args: any[]) => void, seconds: number): () => void
  setImmediate(cb: (...args: any[]) => void): () => void
}
