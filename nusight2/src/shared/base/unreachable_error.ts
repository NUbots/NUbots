/**
 * Represents an error which should never occur within the type system.
 *
 * Most often used for ensuring exhaustive switch cases at compile-time. e.g.
 *
 * <pre>
 * const obj: { type: 'foo' | 'bar' | 'baz' } = { type: 'foo' }
 * switch (obj.type) {
 *   case 'foo': break
 *   case 'bar': break
 *   // Compile-time error, since the 'baz' case is not handled.
 *   default: throw new UnreachableError(obj.type)
 * }
 * </pre>
 */
export class UnreachableError extends Error {
  constructor(x: never) {
    super(`Unhandled case: ${JSON.stringify(x)}`);
  }
}
