import seedrandom from "seedrandom";

/**
 * A seeded pseudo-random number generator. When given the same seed, it will always produce the same set of values.
 *
 * Useful in tests or fakes to generate a predictable range of values that will be the same each time the test is run.
 */
export class SeededRandom {
  constructor(private prng: () => number) {}

  static of(seed: string) {
    return new SeededRandom(seedrandom(seed));
  }

  float(): number {
    return this.prng();
  }

  integer(min: number, max: number): number {
    return Math.floor(this.prng() * (max - min) + min);
  }

  choice<T>(arr: ReadonlyArray<T>): T {
    const index = this.integer(0, arr.length);
    return arr[index];
  }
}
