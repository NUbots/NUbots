// Adapted from @use-it/interval v1.0.0: https://github.com/donavon/use-interval/blob/v1.0.0/src/index.tsx.
// Inlined here as the package has an issue that prevents it from working with Vite.
// See https://github.com/donavon/use-interval/issues/55.
import { useEffect, useRef } from "react";

type Delay = number | null;
type TimerHandler = (...args: any[]) => void;

/**
 * Provides a declarative useInterval
 *
 * @param callback - Function that will be called every `delay` ms.
 * @param delay - Number representing the delay in ms. Set to `null` to "pause" the interval.
 */
export default function useInterval(callback: TimerHandler, delay: Delay) {
  const savedCallbackRef = useRef<TimerHandler>();

  useEffect(() => {
    savedCallbackRef.current = callback;
  }, [callback]);

  useEffect(() => {
    const handler = (...args: any[]) => savedCallbackRef.current!(...args);

    if (delay !== null) {
      const intervalId = setInterval(handler, delay);
      return () => clearInterval(intervalId);
    }
  }, [delay]);
}
