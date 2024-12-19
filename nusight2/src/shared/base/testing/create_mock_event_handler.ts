import { Mock, vi } from "vitest";

export type MockEventHandler<Y extends any[]> = Mock<(handler: (...args: Y) => void) => () => void> & {
  mockEvent(...args: Y): void;
};

// A helper method for triggering event listeners for a mock instance.
// See tests for an example.
export const createMockEventHandler = <Y extends any[]>(): MockEventHandler<Y> => {
  const listeners: Set<(...args: Y) => void> = new Set();
  return Object.assign(
    vi.fn((cb: (...args: Y) => void) => {
      listeners.add(cb);
      return () => {
        listeners.delete(cb);
      };
    }),
    {
      mockEvent: (...args: Y) => {
        for (const listener of listeners) {
          listener(...args);
        }
      },
    },
  );
};
