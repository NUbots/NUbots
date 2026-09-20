/**
 * Parse a CLI argument as a non-negative integer (e.g. a port number or team id). Returns
 * `undefined` if the argument wasn't provided, and also `undefined` (with a warning logged) if it
 * was provided but isn't a finite non-negative whole number, so callers can safely fall back to a
 * default (or treat it as explicitly disabled, for arguments where `0` means "off") rather than
 * e.g. trying to bind a `NaN` port later.
 */
export function parseNonNegativeIntArg(value: unknown, argName: string): number | undefined {
  if (value === undefined) {
    return undefined;
  }

  const parsed = Number(value);
  if (!Number.isFinite(parsed) || !Number.isInteger(parsed) || parsed < 0) {
    console.warn(`Ignoring invalid ${argName} "${value}": expected a non-negative whole number`);
    return undefined;
  }

  return parsed;
}
