import { useCallback, useEffect, useState } from "react";

/**
 * A hook similar to `useState`, but each time the value is set, it is reset back to the initial
 * value after a timeout.
 */
export function useTimedState<T>(initialState: T, timeout: number): [T, (value: T) => void] {
  const [value, setValueInternal] = useState(initialState);
  const [valueKey, setValueKey] = useState(0);

  // Set the value and increment the key to reset the timeout
  const setValue = useCallback(
    (value: T) => {
      setValueInternal(value);
      setValueKey((key) => key + 1);
    },
    [setValueInternal, setValueKey],
  );

  // If the value is different to the default, set a timeout to reset it.
  // This uses the key as dependency so the timeout gets reset when the same value is set multiple times.
  useEffect(() => {
    if (value !== initialState) {
      const id = setTimeout(() => setValue(initialState), timeout);
      return () => clearTimeout(id);
    }
  }, [value, valueKey, timeout]);

  return [value, setValue];
}
