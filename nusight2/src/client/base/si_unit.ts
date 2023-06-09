/**
 * Convert the given value to a tuple of SI value and unit.
 */
export function siUnit(value: number, unit: string = ""): [number, string] {
  if (value == 0) {
    return [0, unit];
  }

  // Work out our SI index between -8 and 8
  const index = Math.max(-8, Math.min(8, Math.floor(Math.log10(Math.abs(value)) / 3)));

  switch (index) {
    case 8:
      return [value * 1e-24, "Y" + unit]; // yotta
    case 7:
      return [value * 1e-21, "Z" + unit]; // zetta
    case 6:
      return [value * 1e-18, "E" + unit]; // exa
    case 5:
      return [value * 1e-15, "P" + unit]; // peta
    case 4:
      return [value * 1e-12, "T" + unit]; // tera
    case 3:
      return [value * 1e-9, "G" + unit]; // giga
    case 2:
      return [value * 1e-6, "M" + unit]; // mega
    case 1:
      return [value * 1e-3, "k" + unit]; // kilo
    case 0:
      return [value, unit]; // base
    case -1:
      return [value * 1e3, "m" + unit]; // milli
    case -2:
      return [value * 1e6, "μ" + unit]; // micro
    case -3:
      return [value * 1e9, "n" + unit]; // nano
    case -4:
      return [value * 1e12, "p" + unit]; // pico
    case -5:
      return [value * 1e15, "f" + unit]; // femto
    case -6:
      return [value * 1e18, "a" + unit]; // atto
    case -7:
      return [value * 1e21, "z" + unit]; // zepto
    case -8:
      return [value * 1e24, "y" + unit]; // yocto
    default:
      return [value, unit];
  }
}
