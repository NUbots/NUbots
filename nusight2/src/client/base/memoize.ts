/**
 * Given a function that takes an object A and returns a B, create a new function which memoizes that A -> B transform.
 *
 * i.e. The first time the memoized function is called with an A, it calculates B using fn(A) and stores B in its
 * internal map. The second time it is called with the same A, it will not call fn(A) and instead just return the B that
 * was created the previous time. Internally the function uses a WeakMap, so B will be automatically garbage collected
 * when its corresponding A no longer exists in memory.
 *
 * e.g.
 * const a = { name: 'Foo' }
 * const fn = a => ({ name: `${a.name}Bar` })
 * const memoizedFn = memoize(fn)
 *
 * fn(a) === f(a) // false
 * memoizedFn(a) === memoizedFn(a) // true
 * memoizedFn(a) // { name: `FooBar` }
 */
export function memoize<A extends object, B>(fn: (a: A) => B): (a: A) => B {
  const map = new WeakMap<A, B>();
  return (a: A) => {
    if (!map.has(a)) {
      map.set(a, fn(a));
    }
    return map.get(a)!;
  };
}

/**
 * Memoize a function by its simple arguments.
 *
 * This is a more flexible version of `memoize()` since it allows the function to have any
 * number of arguments and they can be of any type that is uniquely serializable to a string.
 * Primitives and arrays of primitives are OK. Symbols, objects, and arrays that contain
 * symbols or objects are not. An optional function can be provided to create a unique
 * key from the arguments, in which case the above restrictions do not apply.
 *
 * The downside is that the cache is not garbage collected until the memoized function
 * itself is, so it's best to use this for functions that are called with a small
 * number of unique arguments, where the cache won't grow too large.
 */
export function memoizeBySimpleArgs<T extends (...args: any[]) => any>(
  fn: T,
  createKey?: (...args: Parameters<T>) => string,
): T {
  const cache = new Map<string, ReturnType<T>>();
  return ((...args: Parameters<T>) => {
    const key = createKey ? createKey(...args) : args.join(",");
    const cached = cache.get(key);
    if (cached) {
      return cached;
    }
    const result = fn(...args);
    cache.set(key, result);
    return result;
  }) as T;
}
