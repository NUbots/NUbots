import XXH from "xxhashjs";

/** Hash the given message type using the same algorithm used in NUClear */
export function hashType(type: string): Buffer {
  // Matches hashing implementation from NUClearNet
  // See https://github.com/Fastcode/NUClearNet.js/blob/857ed674/src/NetworkBinding.cpp#L32
  let hashString: string = XXH.h64(type, 0x4e55436c).toString(16);
  // The hash string may truncate if it's smaller than 16 characters so we pad it with 0s
  hashString = ("0".repeat(16) + hashString).slice(-16);

  return Buffer.from((hashString.match(/../g) as string[]).reverse().join(""), "hex");
}
