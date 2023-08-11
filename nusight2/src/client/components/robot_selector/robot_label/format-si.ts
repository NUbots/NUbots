// Adapted from https://github.com/ThomWright/format-si-prefix

const prefixes: { [key: string]: string } = {
  "24": "Y",
  "21": "Z",
  "18": "E",
  "15": "P",
  "12": "T",
  "9": "G",
  "6": "M",
  "3": "k",
  "0": "",
  "-3": "m",
  "-6": "µ",
  "-9": "n",
  "-12": "p",
  "-15": "f",
  "-18": "a",
  "-21": "z",
  "-24": "y",
};

export function formatSI(value: number) {
  if (value === 0) {
    return "0";
  }

  let significand = Math.abs(value);
  let exponent = 0;

  while (significand >= 1000 && exponent < 24) {
    significand /= 1000;
    exponent += 3;
  }

  while (significand < 1 && exponent > -24) {
    significand *= 1000;
    exponent -= 3;
  }

  const signPrefix = value < 0 ? "-" : "";

  if (significand > 1000) {
    return signPrefix + significand.toFixed(0) + prefixes[exponent];
  }

  return signPrefix + parseFloat(significand.toPrecision(3)) + prefixes[exponent];
}
