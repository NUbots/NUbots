import { IComputedValue } from "mobx";
import { onBecomeUnobserved } from "mobx";
import { computed } from "mobx";

function disposableComputedDecorator(target: any, propertyKey: string, descriptor: PropertyDescriptor) {
  if (!descriptor) {
    descriptor = Object.getOwnPropertyDescriptor(target, propertyKey)!;
  }
  const originalMethod = descriptor.get!;
  // Lazily create and store the computed expression per instance.
  // This allows us to bind each computed expression to the correct `this` context.
  const expressionPerInstance = new WeakMap();
  descriptor.get = function () {
    let expr = expressionPerInstance.get(this);
    if (!expr) {
      expr = disposableComputedExpr(originalMethod.bind(this));
      expressionPerInstance.set(this, expr);
    }
    return expr.get();
  };
  return descriptor;
}

/**
 * Designed for creating computed values for managed resources which need to be disposed of after use.
 *
 * Any stale values will be disposed, and the last value will be disposed when the computed is no longer observed.
 */
const disposableComputedExpr = <T extends { dispose(): void }>(fn: () => T): IComputedValue<T> => {
  let latest: T | undefined;
  const expr = computed(() => {
    latest && latest.dispose();
    latest = fn();
    return latest;
  });
  onBecomeUnobserved(expr, () => latest && latest.dispose());
  return expr;
};

export const disposableComputed: DisposableComputed = function computed(arg1: any, arg2: any) {
  if (typeof arg2 === "string") {
    // Used as an annotation e.g. @disposableComputed
    // eslint-disable-next-line
    return disposableComputedDecorator.apply(null, arguments as any);
  }

  // Used as an expression e.g. disposableComputed(() => value)
  return disposableComputedExpr(arg1);
} as any;

export interface DisposableComputed {
  <T extends { dispose(): void }>(func: () => T): IComputedValue<T>;

  (target: Object, key: string | symbol, baseDescriptor?: PropertyDescriptor): void;
}
