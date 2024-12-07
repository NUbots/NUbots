import { vi } from "vitest";

/** Create an event emitter through which mock events can be emitted */
export function createMockEventEmitter() {
  /** The event listeners that have been registered on this emitter, keyed by the event name */
  const allListeners = new Map<string, Set<(...args: any[]) => void>>();

  /** Used to remove an event listener */
  const off = vi
    .fn<(event: string, callback: (...args: any[]) => void) => void>()
    .mockImplementation((event, callback) => {
      const listenersForThisEvent = allListeners.get(event);
      listenersForThisEvent?.delete(callback);
    });

  /** Used to add a new an event listener */
  const on = vi
    .fn<(event: string, callback: (...args: any[]) => void) => () => void>()
    .mockImplementation((event: string, callback) => {
      const listenersForThisEvent = allListeners.get(event) ?? new Set();
      listenersForThisEvent.add(callback);

      allListeners.set(event, listenersForThisEvent);

      return () => {
        off(event, callback);
      };
    });

  /** Used to emit mock events to the registered listeners */
  const emit = (event: string, ...args: any[]) => {
    const listenersForThisEvent = allListeners.get(event);

    if (listenersForThisEvent) {
      for (const listener of listenersForThisEvent) {
        listener(...args);
      }
    }
  };

  return { on, off, emit };
}
