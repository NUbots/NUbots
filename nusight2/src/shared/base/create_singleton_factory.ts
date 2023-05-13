/** Creates a function which will cache the value of the given factory function. */
export const createSingletonFactory = <T>(factory: () => T) => {
  let instance: T;
  return () => {
    if (!instance) {
      instance = factory();
    }
    return instance;
  };
};
