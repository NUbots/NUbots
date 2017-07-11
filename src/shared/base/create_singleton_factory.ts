/**
 * Creates a function which will cache the value of the given factory function.
 * e.g.
 *
 * const fn = createSingleton(() => new MyClass())
 * fn() === fn() // true
 */
export const createSingletonFactory = <T>(factory: () => T) => {
  return (() => {
    let instance: T
    return () => {
      if (!instance) {
        instance = factory()
      }
      return instance
    }
  })()
}
