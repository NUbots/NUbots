import { IComputedValue } from "mobx";
import { observable } from "mobx";
import { autorun } from "mobx";

import { disposableComputed } from "../disposable_computed";

describe("disposableComputed", () => {
  let model: { a: number; b: number };
  let expr: IComputedValue<{ sum: number; dispose: jest.Mock<void, []> }>;

  beforeEach(() => {
    model = observable({ a: 1, b: 1 });
    expr = disposableComputed(() => ({ sum: model.a + model.b, dispose: jest.fn<void, []>() }));
  });

  const countUnique = <T extends unknown>(arr: T[]): number => new Set(arr).size;

  it("returns value on evaluation", () => {
    const value = { foo: "bar", dispose: jest.fn() };
    const expr = disposableComputed(() => value);
    expect(expr.get()).toBe(value);
  });

  describe("when unobserved", () => {
    it("creates new values when repeatably evaluated", () => {
      const someValues = Array.from({ length: 5 }, () => expr.get());
      expect(countUnique(someValues)).toBe(5);
    });

    it("does not dispose value after evaluation", () => {
      const value = expr.get();
      expect(value.dispose).not.toHaveBeenCalled();
    });

    it("disposes stale values when repeatably evaluated", () => {
      const someValues = Array.from({ length: 5 }, () => expr.get());
      const allButLast = someValues.slice(0, -1);
      allButLast.forEach((value) => expect(value.dispose).toHaveBeenCalled());
    });
  });

  describe("when observed", () => {
    let dispose: () => void;

    beforeEach(() => {
      dispose = autorun(() => expr.get());
    });

    it("caches value when repeatably evaluated", () => {
      const someValues = Array.from({ length: 5 }, () => expr.get());
      expect(countUnique(someValues)).toBe(1);
    });

    it("does not dispose value after evaluation", () => {
      const value = expr.get();
      expect(value.dispose).not.toHaveBeenCalled();
    });

    it("disposes stale value after recomputation", () => {
      const value = expr.get();
      model.a++;
      expect(value.dispose).toHaveBeenCalled();
    });

    it("disposes value after disposing observing reaction", () => {
      const value = expr.get();
      dispose();
      expect(value.dispose).toHaveBeenCalled();
    });

    it("creates new values when repeatably evaluated after disposing observing reaction", () => {
      dispose();
      const someValues = Array.from({ length: 5 }, () => expr.get());
      expect(countUnique(someValues)).toBe(5);
    });
  });

  describe("as a decorator", () => {
    let model: Model;
    let viewModel: ViewModel;
    let factory: TriangleFactory;

    class Model {
      @observable.ref color: string;

      constructor({ color }: { color: string }) {
        this.color = color;
      }
    }

    class ViewModel {
      constructor(private readonly factory: TriangleFactory, private readonly model: { color: string }) {}

      @disposableComputed
      get value(): Triangle {
        return this.factory.createTriangle(this.model.color);
      }
    }

    beforeEach(() => {
      factory = new TriangleFactory();
      model = new Model({ color: "red" });
      viewModel = new ViewModel(factory, model);
    });

    it("returns computed value", () => {
      expect(viewModel.value).toBeInstanceOf(Triangle);
      expect(viewModel.value.color).toBe("red");
    });

    it("caches values while being observed", () => {
      const dispose = autorun(() => viewModel.value);
      expect(viewModel.value).toBe(viewModel.value);
      expect(factory.allocations).toBe(1);
      dispose();
    });

    it("disposes when no longer observed", () => {
      const dispose = autorun(() => viewModel.value);
      expect(factory.refCount).toBe(1);
      dispose();
      expect(factory.refCount).toBe(0);
    });

    it("disposes old values as new values are computed", () => {
      const dispose = autorun(() => viewModel.value);
      expect(factory.allocations).toBe(1);
      expect(factory.refCount).toBe(1);

      model.color = "green";
      expect(factory.allocations).toBe(2);
      expect(factory.refCount).toBe(1);

      model.color = "blue";
      expect(factory.allocations).toBe(3);
      expect(factory.refCount).toBe(1);

      dispose();
      expect(factory.allocations).toBe(3);
      expect(factory.refCount).toBe(0);
    });
  });
});

/** Helper classes to track and test allocations and disposes */
class TriangleFactory {
  allocations = 0;
  refCount = 0;

  createTriangle(color: string) {
    this.allocations++;
    this.refCount++;
    return new Triangle(color, () => this.refCount--);
  }
}

class Triangle {
  constructor(public color: string, private onDispose: () => void) {}

  dispose() {
    this.onDispose();
  }
}
