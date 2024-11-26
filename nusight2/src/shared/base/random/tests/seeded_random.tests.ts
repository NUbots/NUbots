import { beforeEach, describe, expect, it } from "vitest";
import { beforeEach, describe, expect, it } from "vitest";
import { range } from "../../range";
import { SeededRandom } from "../seeded_random";

describe("SeededRandom", () => {
  let random: SeededRandom;

  beforeEach(() => {
    random = SeededRandom.of("random_seed");
  });

  describe("#integer", () => {
    it("should always generate the same set of values", () => {
      const numbers = range(100).map(() => random.integer(0, 100));
      expect(numbers).toMatchSnapshot();
    });

    it("should generate different sets of values for a different seeds", () => {
      const randomA = SeededRandom.of("random_seed_a");
      const randomB = SeededRandom.of("random_seed_b");

      const numbersA = range(100).map(() => randomA.integer(0, 100));
      const numbersB = range(100).map(() => randomB.integer(0, 100));

      expect(numbersA).not.toEqual(numbersB);
    });

    it("should always generate values within the given range", () => {
      const numbers = range(1000).map(() => random.integer(0, 5));

      const withinRange = numbers.every((n) => n >= 0 && n < 5);
      expect(withinRange).toBeTruthy();
    });

    it("should generally produce values across the range", () => {
      const numbers = range(50).map(() => random.integer(0, 5));

      expect(numbers.some((n) => n === 0)).toBeTruthy();
      expect(numbers.some((n) => n === 1)).toBeTruthy();
      expect(numbers.some((n) => n === 2)).toBeTruthy();
      expect(numbers.some((n) => n === 3)).toBeTruthy();
      expect(numbers.some((n) => n === 4)).toBeTruthy();
    });
  });

  describe("#float", () => {
    it("should always generate the same set of values", () => {
      const numbers = range(100).map(() => random.float());
      expect(numbers).toMatchSnapshot();
    });

    it("should always generate values within [0,1)", () => {
      const numbers = range(1000).map(() => random.float());

      const withinRange = numbers.every((n) => n >= 0 && n < 1);
      expect(withinRange).toBeTruthy();
    });
  });

  describe("#choice", () => {
    it("should always return the same element from the array", () => {
      const list = range(100);
      const choice = random.choice(list);
      expect(choice).toMatchSnapshot();
    });
  });
});
