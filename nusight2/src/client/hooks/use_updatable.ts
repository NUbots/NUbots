import { useRef, useState } from "react";

/**
 * Create a memoized value that can be updated in place when dependencies change. Works as follows:
 *   - calls `create()` on first render and stores the returned value
 *   - when dependencies change, calls either `create()` or `update()`:
 *     - calls `update()` if the existing value is not null or undefined
 *     - calls `create()` otherwise
 */
export function useUpdatable<V, T extends [...any[]]>(
  create: (dependencies: [...T]) => V,
  update: (value: NonNullable<V>, newDependencies: [...T]) => void,
  dependencies: [...T],
) {
  // Remember the last state of the dependencies
  const prevDependencies = useRef(dependencies);

  // Create the initial value
  const [value, setValue] = useState(() => create(dependencies));

  // Check if the dependencies have changed
  const needsUpdate =
    dependencies.length !== prevDependencies.current.length ||
    dependencies.some((dep, i) => !Object.is(dep, prevDependencies.current[i]));

  // Store the new dependencies
  prevDependencies.current = dependencies;

  // Update the value using the new dependencies if there was a change
  if (needsUpdate) {
    if (value !== null && value !== undefined) {
      update(value, dependencies);
    } else {
      setValue(create(dependencies));
    }
  }

  return value;
}
