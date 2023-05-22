/** Parse the given event string in the form `type#subtype` to extract the type and subtype. */
export function parseEventString(
  event: string,
  opts?: { throwOnInvalidSubtype?: boolean },
): { type: string; subtype: number | undefined } {
  const [type, subtypeStr] = event.split("#");
  const subtype = subtypeStr ? parseInt(subtypeStr, 10) : undefined;

  if (Number.isNaN(subtype)) {
    if (opts?.throwOnInvalidSubtype ?? true) {
      throw new Error(`invalid subtype in event "${event}". subtype "${subtypeStr}" is not a number`);
    } else {
      return { type, subtype: undefined };
    }
  }

  return { type, subtype };
}
