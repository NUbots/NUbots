import { IComputedValue } from 'mobx'
import { onBecomeUnobserved } from 'mobx'
import { computed } from 'mobx'

/**
 * Designed for creating computed values for managed resources which need to be disposed of after use.
 *
 * Any stale values will be disposed, and the last value will be disposed when the computed is no longer observed.
 */
export const disposableComputed = <T extends { dispose(): void; }>(fn: () => T): IComputedValue<T> => {
  let latest: T | undefined
  const expr = computed(() => {
    latest && latest.dispose()
    latest = fn()
    return latest
  })
  onBecomeUnobserved(expr, () => latest && latest.dispose())
  return expr
}
