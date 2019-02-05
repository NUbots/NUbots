import { onBecomeUnobserved } from 'mobx'
import { computed } from 'mobx'
import { IComputedValue } from 'mobx'

export const createUpdatableComputed = <T, O>(
  create: (opts: O) => T,
  update: (instance: T, opts: O) => void,
  dispose?: (instance: T) => void,
) => (getOpts: () => O): IComputedValue<T> => {
  let instance: T | undefined
  const expr = computed(() => {
    const opts = getOpts()
    if (instance == null) {
      instance = create(opts)
    }
    update(instance, opts)
    return instance
  }, { equals: () => false })
  onBecomeUnobserved(expr, () => {
    dispose && instance && dispose(instance)
    instance = undefined
  })
  return expr
}
