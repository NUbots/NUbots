import { Clock } from "../../shared/time/clock";
import { CancelTimer } from "../../shared/time/clock";

const SecondsToMilliseconds = 1e3;
const MillisecondsToSeconds = 1e-3;
const NanosecondsToSeconds = 1e-9;

function setTimeout(cb: () => void, seconds: number): CancelTimer {
  const handle = global.setTimeout(cb, seconds * SecondsToMilliseconds);
  return () => global.clearTimeout(handle);
}

function setInterval(cb: () => void, seconds: number): CancelTimer {
  const handle = global.setInterval(cb, seconds * SecondsToMilliseconds);
  return () => global.clearInterval(handle);
}

function nextTick(cb: () => void): void {
  process.nextTick(cb);
}

function performanceNow(): number {
  const t = process.hrtime();
  return t[0] + t[1] * NanosecondsToSeconds;
}

export const NodeSystemClock: Clock = {
  now: () => Date.now() * MillisecondsToSeconds,
  date: () => new Date(),
  performanceNow,
  setTimeout,
  setInterval,
  nextTick,
};
