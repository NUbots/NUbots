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
