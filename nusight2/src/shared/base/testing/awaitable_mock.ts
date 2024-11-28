import { type Mock, vi } from "vitest";

import { WaitForWrappedFn, wrapFnToAwaitCalls } from "./wrap_fn_to_await_calls";

type MockProcedure = (...args: any[]) => any;

export type AwaitableMock<T extends MockProcedure> = Mock<T> & {
  waitForCall: WaitForWrappedFn<T>;
  waitForCalls: (count: number, timeout?: number) => ReturnType<WaitForWrappedFn<T>>;
};

/**
 * Create a mock function that provides a `waitForCall` method to await calls to the mock.
 *
 * Note: `mockImplementation()` should not be called on the mock returned by this function,
 * as that will override the mechanism that allows awaiting calls to the mock. Instead,
 * the mock implementation should be passed to this function.
 */
export function createAwaitableMock<T extends MockProcedure = (...args: any[]) => void>(
  implementation: T = (() => {}) as any,
): AwaitableMock<T> {
  const [implementationWrapped, waitForCall] = wrapFnToAwaitCalls(implementation);

  const mock = vi.fn<T>(implementationWrapped);

  const extensions = {
    waitForCall,
    waitForCalls: (count: number, timeout?: number) => waitForCall(undefined, { count, timeout }),
  };

  return Object.assign(mock, extensions);
}
