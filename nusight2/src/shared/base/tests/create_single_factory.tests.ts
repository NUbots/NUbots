import { createSingletonFactory } from "../create_singleton_factory";

describe("createSingletonFactory", () => {
  class MyClass {}

  it("returns a new instance from factory", () => {
    const factory = createSingletonFactory(() => new MyClass());
    expect(factory()).toBeInstanceOf(MyClass);
  });

  it("returns identical instance from same factory", () => {
    const factory = createSingletonFactory(() => new MyClass());
    expect(factory()).toBe(factory());
  });

  it("returns separate instances from separate factories", () => {
    const factoryA = createSingletonFactory(() => new MyClass());
    const factoryB = createSingletonFactory(() => new MyClass());
    expect(factoryA()).not.toBe(factoryB());
  });
});
