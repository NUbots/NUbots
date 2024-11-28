import { describe, expect, it } from "vitest";

import { formatSI } from "../format-si";

describe("formatSI", () => {
  it("formats yotta numbers", () => {
    expect(formatSI(1e24)).toEqual("1Y");
    expect(formatSI(-1e24)).toEqual("-1Y");
    expect(formatSI(1230000000000000000000000)).toEqual("1.23Y");
  });

  it("formats zetta numbers", () => {
    expect(formatSI(1e21)).toEqual("1Z");
    expect(formatSI(-1e21)).toEqual("-1Z");
    expect(formatSI(1230000000000000000000)).toEqual("1.23Z");
  });

  it("formats exa numbers", () => {
    expect(formatSI(1e18)).toEqual("1E");
    expect(formatSI(-1e18)).toEqual("-1E");
    expect(formatSI(1230000000000000000)).toEqual("1.23E");
  });

  it("formats peta numbers", () => {
    expect(formatSI(1e15)).toEqual("1P");
    expect(formatSI(-1e15)).toEqual("-1P");
    expect(formatSI(1230000000000000)).toEqual("1.23P");
  });

  it("formats tera numbers", () => {
    expect(formatSI(1e12)).toEqual("1T");
    expect(formatSI(-1e12)).toEqual("-1T");
    expect(formatSI(1230000000000)).toEqual("1.23T");
  });

  it("formats giga numbers", () => {
    expect(formatSI(1e9)).toEqual("1G");
    expect(formatSI(-1e9)).toEqual("-1G");
    expect(formatSI(1230000000)).toEqual("1.23G");
  });

  it("formats mega numbers", () => {
    expect(formatSI(1e6)).toEqual("1M");
    expect(formatSI(-1e6)).toEqual("-1M");
    expect(formatSI(1230000)).toEqual("1.23M");
  });

  it("formats kilo numbers", () => {
    expect(formatSI(1e3)).toEqual("1k");
    expect(formatSI(-1e3)).toEqual("-1k");
    expect(formatSI(1230)).toEqual("1.23k");
  });

  it("returns integers between [-999, 999] unformatted", () => {
    expect(formatSI(999)).toEqual("999");
    expect(formatSI(-999)).toEqual("-999");
    expect(formatSI(0)).toEqual("0");
    expect(formatSI(-0)).toEqual("0");
    expect(formatSI(1)).toEqual("1");
    expect(formatSI(-1)).toEqual("-1");
  });

  it("formats milli numbers", () => {
    expect(formatSI(1e-3)).toEqual("1m");
    expect(formatSI(-1e-3)).toEqual("-1m");
    expect(formatSI(0.123)).toEqual("123m");
  });

  it("formats micro numbers", () => {
    expect(formatSI(1e-6)).toEqual("1µ");
    expect(formatSI(-1e-6)).toEqual("-1µ");
    expect(formatSI(0.000123)).toEqual("123µ");
  });

  it("formats nano numbers", () => {
    expect(formatSI(1e-9)).toEqual("1n");
    expect(formatSI(-1e-9)).toEqual("-1n");
    expect(formatSI(0.000000123)).toEqual("123n");
  });

  it("formats picko numbers", () => {
    expect(formatSI(1e-12)).toEqual("1p");
    expect(formatSI(-1e-12)).toEqual("-1p");
    expect(formatSI(0.000000000123)).toEqual("123p");
  });

  it("formats fempto numbers", () => {
    expect(formatSI(1e-15)).toEqual("1f");
    expect(formatSI(-1e-15)).toEqual("-1f");
    expect(formatSI(0.000000000000123)).toEqual("123f");
  });

  it("formats atto numbers", () => {
    expect(formatSI(1e-18)).toEqual("1a");
    expect(formatSI(-1e-18)).toEqual("-1a");
    expect(formatSI(0.000000000000000123)).toEqual("123a");
  });

  it("formats zepto numbers", () => {
    expect(formatSI(1e-21)).toEqual("1z");
    expect(formatSI(-1e-21)).toEqual("-1z");
    expect(formatSI(0.000000000000000000123)).toEqual("123z");
  });

  it("formats yocto numbers", () => {
    expect(formatSI(1e-24)).toEqual("1y");
    expect(formatSI(-1e-24)).toEqual("-1y");
    expect(formatSI(0.000000000000000000000123)).toEqual("123y");
  });
});
