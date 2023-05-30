import { SeededRandom } from "../../../shared/base/random/seeded_random";
import { toTimestamp } from "../timestamp";
import { toSeconds } from "../timestamp";

describe("protobuf timestamp", () => {
  let random: SeededRandom;

  beforeEach(() => {
    random = SeededRandom.of("timestamp");
  });

  it("converts timestamps to protocol buffers", () => {
    const value = toTimestamp(1000.0001);

    expect(value.seconds).toBeCloseTo(1000);
    expect(value.nanos).toBeCloseTo(100000);
  });

  it("converts protocol buffers to timestamps", () => {
    const value = toSeconds({
      seconds: 12345,
      nanos: 54321,
    });

    expect(value).toBeCloseTo(12345.000054321);
  });

  it("performs roundtrips between protocol buffers and timestamps", () => {
    for (let i = 0; i < 10; i++) {
      const t = random.float() * 1e5;

      expect(toSeconds(toTimestamp(t))).toBeCloseTo(t);
    }
  });
});
