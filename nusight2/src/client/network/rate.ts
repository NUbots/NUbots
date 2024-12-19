import { action } from "mobx";
import { observable } from "mobx";
import { throttle } from "throttle-debounce";

import { Clock } from "../../shared/time/clock";
import { BrowserSystemClock } from "../time/browser_clock";

type RateOpts = {
  smoothing: number;
};

export class Rate {
  @observable rate = 0;
  private smoothing: number;
  private currentRate: number = 0;
  private currentUpdateCount: number = 0;
  private lastUpdateTime?: number;

  constructor(
    { smoothing }: RateOpts,
    private clock: Clock,
  ) {
    this.smoothing = smoothing;
  }

  static of(opts: RateOpts) {
    return new Rate(opts, BrowserSystemClock);
  }

  update(newCount: number) {
    // We floor here to be able to compare `now` and `this.lastUpdateTime`
    // as ints below. Otherwise they're almost always different due to the
    // decimal places and issues with floating point equality.
    const now = Math.floor(this.clock.now());

    if (this.lastUpdateTime === undefined) {
      this.currentUpdateCount = newCount;
      this.lastUpdateTime = now;
    } else if (this.lastUpdateTime === now) {
      this.currentUpdateCount += newCount;
    } else {
      // Update the current rate using the culmulative sum for the ending time period
      this.currentRate =
        this.currentRate * this.smoothing +
        (this.currentUpdateCount / (now - this.lastUpdateTime)) * (1.0 - this.smoothing);

      // Commit the current rate at a throttled interval
      this.commit(this.currentRate);

      // Start a new culmulative sum for the next rate update
      this.currentUpdateCount = newCount;

      this.lastUpdateTime = now;
    }
  }

  private commit = throttle(
    1000,
    action((rate: number) => (this.rate = rate)),
  );
}
