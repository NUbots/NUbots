import { computed } from "mobx";
import { IComputedValue } from "mobx";
import { onBecomeUnobserved } from "mobx";
import { DeepMap } from "mobx-utils/lib/deepMap";

export const createUpdatableComputed = <T, O>(
  create: (opts: O) => T,
  update: (instance: T, opts: O) => void,
  dispose?: (instance: T) => void,
) => {
  return <D extends unknown[]>(getOpts: (...deps: D) => O): ((...deps: D) => T) => {
    const cache = new DeepMap<IComputedValue<T>>();
    return (...deps: D): T => {
      const entry = cache.entry(deps);
      if (entry.exists()) {
        return entry.get().get();
      }
      let instance: T | undefined;
      const expr = computed(
        () => {
          const opts = getOpts(...deps);
          if (instance == null) {
            instance = create(opts);
          }
          update(instance, opts);
          return instance;
        },
        { equals: () => false },
      );
      entry.set(expr);
      onBecomeUnobserved(expr, () => {
        instance != null && dispose && dispose(instance);
        cache.entry(deps).delete();
      });
      return expr.get();
    };
  };
};
