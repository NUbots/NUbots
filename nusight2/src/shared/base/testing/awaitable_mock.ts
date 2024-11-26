import { vi } from "vitest";
import { WaitForWrappedFn, wrapFnToAwaitCalls } from "./wrap_fn_to_await_calls";

export type AwaitableMock<MockReturn, MockArgs extends any[]> = jest.Mock<MockReturn, MockArgs> & {
  waitForCall: WaitForWrappedFn<jest.Mock<MockReturn, MockArgs>>;
  waitForCalls: (count: number, timeout?: number) => ReturnType<WaitForWrappedFn<jest.Mock<MockReturn, MockArgs>>>;
};

/**
 * Create a Jest mock function that provides a `waitForCall` method to await calls to the mock.
 *
 * Note: `mockImplementation()` should not be called on the mock returned by this function,
 * as that will override the mechanism that allows awaiting calls to the mock. Instead,
 * the mock implementation should be passed to this function.
 */
export function createAwaitableMock<MockReturn = void, MockArgs extends any[] = any[]>(
  implementation: (...args: MockArgs) => MockReturn = (() => {}) as any,
): AwaitableMock<MockReturn, MockArgs> {
  const [implementationWrapped, waitForCall] = wrapFnToAwaitCalls(implementation);

  return Object.assign(vi.fn<MockReturn, MockArgs>(implementationWrapped), {
    waitForCall,
    waitForCalls: (count: number, timeout?: number) => waitForCall(undefined, { count, timeout }),
  });
}
