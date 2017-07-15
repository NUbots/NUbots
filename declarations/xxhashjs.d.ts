declare module 'xxhashjs' {
  type XXHashJS = {
    h32(data: string, seed: number): XXHashJS
    h64(data: string, seed: number): XXHashJS
    toString(radix: number): string
  }
  const value: XXHashJS
  export = value
}
