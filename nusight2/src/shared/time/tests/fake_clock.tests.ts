import { beforeEach, describe, expect, it } from "vitest";
import { beforeEach, describe, expect, it, vi } from "vitest";

import { FakeClock } from "../fake_clock";

describe("FakeClock", () => {
  let clock: FakeClock;

  beforeEach(() => {
    clock = FakeClock.of();
  });

  describe("#now", () => {
    it("starts at 0 by default", () => {
      expect(FakeClock.of().now()).toEqual(0);
    });

    it("returns time with initialized value", () => {
      expect(FakeClock.of(1000).now()).toEqual(1000);
    });
  });

  describe("#date", () => {
    it("starts at unix epoch by default", () => {
      expect(FakeClock.of().date().toUTCString()).toEqual("Thu, 01 Jan 1970 00:00:00 GMT");
    });

    it("returns date at the initialized time", () => {
      expect(FakeClock.of(1492778724).date().toUTCString()).toEqual("Fri, 21 Apr 2017 12:45:24 GMT");
    });
  });

  describe("#tick", () => {
    it("moves forward time", () => {
      expect(clock.now()).toEqual(0);
      expect(clock.performanceNow()).toEqual(0);

      clock.tick(10);
      expect(clock.now()).toEqual(10);
      expect(clock.performanceNow()).toEqual(10);

      clock.tick(10);
      expect(clock.now()).toEqual(20);
      expect(clock.performanceNow()).toEqual(20);

      clock.tick(10);
      expect(clock.now()).toEqual(30);
      expect(clock.performanceNow()).toEqual(30);
    });
  });

  describe("#setTimeout", () => {
    it("does not invoke callback synchronously", () => {
      const spy = vi.fn();
      clock.setTimeout(spy, 0);
      expect(spy).not.toHaveBeenCalled();
    });

    it("does not invoke callback before expected time", () => {
      const spy = vi.fn();
      clock.setTimeout(spy, 2);
      clock.tick(1);
      expect(spy).not.toHaveBeenCalled();
    });

    it("invokes callback at expected time", () => {
      const spy = vi.fn();
      clock.setTimeout(spy, 2);
      clock.tick(2);
      expect(spy).toHaveBeenCalled();
    });

    it("does not invoke callback when cancelled", () => {
      const spy = vi.fn();
      const cancel = clock.setTimeout(spy, 2);

      cancel();
      clock.tick(100);
      expect(spy).not.toHaveBeenCalled();
    });
  });

  describe("#setInterval", () => {
    it("does not invoke callback synchronously", () => {
      const spy = vi.fn();
      clock.setInterval(spy, 2);
      expect(spy).not.toHaveBeenCalled();
    });

    it("invokes callback regularly at expected times", () => {
      const spy = vi.fn();
      clock.setInterval(spy, 2);

      clock.tick(2);
      expect(spy).toHaveBeenCalledTimes(1);

      clock.tick(1); // Tick half way, should not call yet.
      expect(spy).toHaveBeenCalledTimes(1);

      clock.tick(1); // Tick all the way, should be called now.
      expect(spy).toHaveBeenCalledTimes(2);
    });

    it("does not invoke callback when cancelled", () => {
      const spy = vi.fn();
      const cancel = clock.setInterval(spy, 2);

      cancel();
      clock.tick(100);
      expect(spy).not.toHaveBeenCalled();
    });
  });

  describe("#nextTick", () => {
    it("does not invoke callback synchronously", () => {
      const spy = vi.fn();
      clock.nextTick(spy);
      expect(spy).not.toHaveBeenCalled();
    });

    it("invokes callback after any tick", () => {
      const spy = vi.fn();
      clock.nextTick(spy);

      clock.tick(0.1);
      expect(spy).toHaveBeenCalled();
    });

    it("does not invoke callback when cancelled", () => {
      const spy = vi.fn();
      const cancel = clock.nextTick(spy);

      cancel();
      clock.tick(100);
      expect(spy).not.toHaveBeenCalled();
    });
  });

  describe("#runAllTimers", () => {
    it("invokes all scheduled timers", () => {
      const spy1 = vi.fn();
      const spy2 = vi.fn();
      const spy3 = vi.fn();

      clock.setTimeout(spy1, 1);
      clock.setTimeout(spy2, 10);
      clock.setTimeout(spy3, 100);
      clock.runAllTimers();

      expect(spy1).toHaveBeenCalled();
      expect(spy2).toHaveBeenCalled();
      expect(spy3).toHaveBeenCalled();
    });

    it("invokes all scheduled timers, including any new timers that were scheduled from running them", () => {
      const spy1 = vi.fn();
      const spy2 = vi.fn();

      clock.setTimeout(() => {
        spy1();
        clock.setTimeout(spy2, 1);
      }, 1);

      clock.runAllTimers();
      expect(spy1).toHaveBeenCalled();
      expect(spy2).toHaveBeenCalled();
    });

    it("throws an exception where there are too many scheduled tasks", () => {
      for (let i = 0; i < 2000; i++) {
        clock.setTimeout(vi.fn(), 1);
      }

      expect(() => clock.runAllTimers()).toThrow(/Exceeded clock task limit/);
    });
  });

  describe("#runOnlyPendingTimers", () => {
    it("invokes all scheduled timers", () => {
      const spy1 = vi.fn();
      const spy2 = vi.fn();
      const spy3 = vi.fn();

      clock.setTimeout(spy1, 1);
      clock.setTimeout(spy2, 10);
      clock.setTimeout(spy3, 100);

      clock.runOnlyPendingTimers();
      expect(spy1).toHaveBeenCalled();
      expect(spy2).toHaveBeenCalled();
      expect(spy3).toHaveBeenCalled();
    });

    it("invokes all scheduled timers, excluding any new timers that were scheduled from running them", () => {
      const spy1 = vi.fn();
      const spy2 = vi.fn();
      const spy3 = vi.fn();

      clock.setTimeout(() => {
        spy1();
        clock.setTimeout(spy3, 20);
      }, 10);

      clock.setTimeout(spy2, 10);

      clock.runOnlyPendingTimers();
      expect(spy1).toHaveBeenCalled();
      expect(spy2).toHaveBeenCalled();
      expect(spy3).not.toHaveBeenCalled();
    });

    it("throws an exception where there are too many scheduled tasks", () => {
      for (let i = 0; i < 2000; i++) {
        clock.setTimeout(vi.fn(), 1);
      }

      expect(() => clock.runOnlyPendingTimers()).toThrow(/Exceeded clock task limit/);
    });
  });

  describe("#runTimersToTime", () => {
    it("invokes all scheduled timers before the given time", () => {
      const spy1 = vi.fn();
      const spy2 = vi.fn();
      const spy3 = vi.fn();

      clock.setTimeout(spy1, 1);
      clock.setTimeout(spy2, 10);
      clock.setTimeout(spy3, 100);

      clock.runTimersToTime(50);
      expect(spy1).toHaveBeenCalled();
      expect(spy2).toHaveBeenCalled();
      expect(spy3).not.toHaveBeenCalled();
    });

    it("throws an exception where there are too many scheduled tasks", () => {
      for (let i = 0; i < 2000; i++) {
        clock.setTimeout(vi.fn(), 1);
      }

      expect(() => clock.runTimersToTime(2000)).toThrow(/Exceeded clock task limit/);
    });
  });
});
