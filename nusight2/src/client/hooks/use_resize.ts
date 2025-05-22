import { useCallback, useEffect, useLayoutEffect } from "react";
import { debounce } from "throttle-debounce";

/** React to an element resizing with a callback */
export function useResize<T extends HTMLElement>(ref: React.RefObject<T | null>, cb: (element: T) => void) {
  useLayoutEffect(() => {
    const element = ref.current;

    if (!element) {
      console.warn("useResize: unable to setup resize observer: ref to element not available when effect ran");
      return;
    }

    const resizeObserver = new ResizeObserver(() => cb(element));
    resizeObserver.observe(element);
    return () => {
      resizeObserver.disconnect();
    };
  }, [cb]);
}

/**
 * React to an element resizing with a callback that is debounced so it's only
 * called at the end of the resize.
 *
 * @param delay The delay for the debouncing in milliseconds. Default value is 100.
 */
export function useResizeDebounced<T extends HTMLElement>(
  ref: React.RefObject<T | null>,
  cb: (element: T) => void,
  delay?: number,
) {
  const callback = useCallback(debounce(delay ?? 100, cb), [cb, delay]);
  useEffect(() => {
    const element = ref.current;

    if (!element) {
      console.warn("useResizeDebounced: unable to setup resize observer: ref to element not available when effect ran");
      return;
    }

    const resizeObserver = new ResizeObserver(() => callback(element));
    resizeObserver.observe(element);
    return () => {
      resizeObserver.disconnect();
    };
  }, [callback]);
}
