import { CancelTimer } from "../../shared/time/clock";
import { Clock } from "../../shared/time/clock";

const SecondsToMilliseconds = 1e3;
const MillisecondsToSeconds = 1e-3;

function setTimeout(cb: () => void, seconds: number): CancelTimer {
  const handle = window.setTimeout(cb, seconds * SecondsToMilliseconds);
  return () => window.clearTimeout(handle);
}

function setInterval(cb: () => void, seconds: number): CancelTimer {
  const handle = window.setInterval(cb, seconds * SecondsToMilliseconds);
  return () => window.clearInterval(handle);
}

function nextTick(cb: () => void): void {
  // Promises are guaranteed to resolve asynchronously. This is a faster alternative to setTimeout(cb, 0).
  // See https://jakearchibald.com/2015/tasks-microtasks-queues-and-schedules/
  Promise.resolve().then(cb);
}

function performanceNow(): number {
  return window.performance.now() * MillisecondsToSeconds;
}

export const BrowserSystemClock: Clock = {
  now: () => Date.now() * MillisecondsToSeconds,
  date: () => new Date(),
  performanceNow,
  setTimeout,
  setInterval,
  nextTick,
};
