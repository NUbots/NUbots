/** Any callable function*/
type Callable = (...args: any[]) => any;

/** Information about a call to a function: input is arguments, output is return */
type CallableCall<T extends Callable> = { input: Parameters<T>; output: ReturnType<T> };

/** A callback that can be registered to be called when a function is called */
type EmitterCallback<T extends Callable> = (callInfo: CallableCall<T>) => void;

/** A class that can be used to wrap a function and emit events when it is called */
class WrappedFnEmitter<T extends Callable = any> {
  callbacks = new Set<EmitterCallback<T>>();

  addListener = (callback: EmitterCallback<T>) => {
    this.callbacks.add(callback);
  };

  removeListener = (callback: EmitterCallback<T>) => {
    this.callbacks.delete(callback);
  };

  call = (callInfo: CallableCall<T>) => {
    for (const callback of this.callbacks) {
      callback(callInfo);
    }
  };
}

/**
 * A function that returns a promise which resolves when the wrapped function is called, or rejects after a timeout.
 * A callback can be passed to do work while waiting for the call. A timeout in milliseconds can be specified via
 * `opts.timeout`.
 */
export type WaitForWrappedFn<T extends Callable> = (
  doWorkWhileWaiting?: () => void | Promise<void>,
  opts?: { timeout?: number; count?: number },
) => Promise<CallableCall<T>[]>;

/** The result of wrapping a function to await calls to it */
export type WrapFnResult<T extends Callable> = [
  /** The wrapped function. This should be used in place of the original function to detect calls to it. */
  wrappedFn: T,

  /** A function that can be used to wait for calls to the wrapped function. */
  waitForCall: WaitForWrappedFn<T>,
];

/**
 * Wraps a function to emit events when it is called, and returns a tuple of the wrapped function
 * and a function that can be used to await calls to it.
 */
export function wrapFnToAwaitCalls<T extends Callable>(fn: T): WrapFnResult<T> {
  const emitter = new WrappedFnEmitter();

  const wrappedFn = (...input: Parameters<T>) => {
    const output = fn(...input);

    // Queue the call to the emitter for after the wrapped function returns
    setImmediate(emitter.call, { input, output });

    return output;
  };

  const waitForCall: WrapFnResult<T>["1"] = async (doWorkWhileWaiting, opts = {}) => {
    const timeout = opts.timeout ?? 250;

    const expectedCallCount = opts.count ?? 1;
    let currentCallCount = opts.count ?? 1;

    const calls: CallableCall<T>[] = [];

    // Set up a promise that resolves when the wrapped function is called and start the wait timeout
    const waitPromise = new Promise<CallableCall<T>[]>((resolve, reject) => {
      let timeoutId: NodeJS.Timeout | undefined = undefined;

      const onCall = (callInfo: CallableCall<T>) => {
        calls.push(callInfo);

        if (currentCallCount > 1) {
          currentCallCount--;
          return;
        }

        if (timeoutId) {
          clearTimeout(timeoutId);
          timeoutId = undefined;
        }

        emitter.removeListener(onCall);
        resolve(calls);
      };

      timeoutId = setTimeout(() => {
        emitter.removeListener(onCall);
        const error =
          expectedCallCount > 1
            ? new Error(
                `Timed out waiting for wrapped function to be called. ` +
                  `Expected ${expectedCallCount} calls but got ${expectedCallCount - currentCallCount}`,
              )
            : new Error("Timed out waiting for wrapped function to be called");
        reject(error);
      }, timeout);

      emitter.addListener(onCall);
    });

    // Do work while waiting for the call. The work here will probably be what triggers the call.
    await doWorkWhileWaiting?.();

    // Return the promise that resolves when the wrapped function is called
    return waitPromise;
  };

  return [wrappedFn as T, waitForCall];
}
