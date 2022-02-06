/** Take an array of functions and return a function that calls them all. */
export function compose(fns: (() => void)[]): () => void {
  return () => {
    for (const fn of fns) {
      fn()
    }
  }
}
