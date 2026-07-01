import { useRef, useState } from "react";

/**
 * Map properties from a dependency object to an instance object. Works as follows:
 *  - calls `create()` on first render and stores the returned value as the instance object
 *  - when properties of `dependency` change, calls `update()` with the instance object and dependency object
 *  - returns the instance
 */
export function usePropertyChange<D extends object, T extends object>(
  create: (initialValue: D) => T,
  update: (instance: T, newValue: D) => void,
  dependency: D,
) {
  // Last values of the dependency object
  const prevDependencies = useRef(Object.values(dependency));

  // Create the initial instance object
  const [instance] = useState(() => create(dependency));

  // Check if the properties of the dependency object have changed
  const newDependencies = Object.values(dependency);
  const needsUpdate =
    prevDependencies.current.length !== newDependencies.length ||
    prevDependencies.current.some((dep, i) => !Object.is(dep, newDependencies[i]));

  // Store the new values of the dependency object
  prevDependencies.current = newDependencies;

  // Update the instance object with the new values
  if (needsUpdate) {
    update(instance, dependency);
  }

  return instance;
}
