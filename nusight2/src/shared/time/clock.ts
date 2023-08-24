export interface Clock {
  now(): number;

  date(): Date;

  performanceNow(): number;

  setTimeout(cb: () => void, seconds: number): CancelTimer;

  setInterval(cb: () => void, seconds: number): CancelTimer;

  nextTick(cb: () => void): void;
}

export type CancelTimer = () => void;
