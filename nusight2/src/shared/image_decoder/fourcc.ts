/**
 * Convert a four letter string into its integer fourcc code (see http://fourcc.org/)
 * This code allows identification of a stream using the integer.
 *
 * @param code four letters that describe the format
 *
 * @return the fourcc integer code for this format
 */
export function fourcc(code: string): number {
  return (code.charCodeAt(3) << 24) | (code.charCodeAt(2) << 16) | (code.charCodeAt(1) << 8) | code.charCodeAt(0);
}

export function fourccToString(code: number): string {
  return (
    String.fromCharCode(code & 0xff) +
    String.fromCharCode((code >> 8) & 0xff) +
    String.fromCharCode((code >> 16) & 0xff) +
    String.fromCharCode((code >> 24) & 0xff)
  );
}
