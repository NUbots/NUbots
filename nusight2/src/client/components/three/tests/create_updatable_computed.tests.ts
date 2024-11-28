import { computed } from "mobx";
import { observe } from "mobx";
import { observable } from "mobx";
import { Mock } from 'vitest'
import { beforeEach, describe, expect, it, vi } from "vitest";

import { createUpdatableComputed } from "../create_updatable_computed";

describe("createUpdatableComputed", () => {
  let model: Model;
  let viewModel: ViewModel;
  let onChange: Mock;

  const computedTriangle = createUpdatableComputed(
    (opts: TriangleOpts) => new Triangle(opts),
    (triangle, { id, color }) => {
      triangle.id = id;
      triangle.color = color;
    },
    (triangle) => triangle.dispose(),
  );

  class Model {
    @observable.ref color: string;

    constructor({ color }: { color: string }) {
      this.color = color;
    }
  }

  class ViewModel {
    constructor(private readonly model: { color: string }) {}

    readonly triangle = computedTriangle((id: string) => ({
      id,
      color: this.model.color,
    }));
  }

  type TriangleOpts = { color: string; id: string };

  class Triangle {
    id: string;
    color: string;
    disposed: boolean = false;

    constructor({ id, color }: TriangleOpts) {
      this.id = id;
      this.color = color;
    }

    dispose() {
      this.disposed = true;
    }
  }

  beforeEach(() => {
    model = new Model({ color: "red" });
    viewModel = new ViewModel(model);
    onChange = vi.fn();
  });

  it("returns computed value", () => {
    const triangle = viewModel.triangle("foo");
    expect(triangle).toBeInstanceOf(Triangle);
    expect(triangle.color).toBe("red");
  });

  it("caches value with identical arguments", () => {
    expect(viewModel.triangle("foo")).toBe(viewModel.triangle("foo"));
  });

  it("does not caches value with unique arguments", () => {
    expect(viewModel.triangle("foo")).not.toBe(viewModel.triangle("bar"));
  });

  it("maintains reference as its updated", () => {
    const expr = computed(() => viewModel.triangle("foo"), { equals: () => false });
    const dispose = observe(expr, onChange);

    const firstTriangle = viewModel.triangle("foo");
    model.color = "green";
    expect(firstTriangle.color).toBe("green");

    const secondTriangle = viewModel.triangle("foo");
    model.color = "blue";
    expect(secondTriangle.color).toBe("blue");

    expect(secondTriangle).toBe(firstTriangle);
    expect(onChange).toBeCalledTimes(2);
    dispose();
  });

  it("does not dispose when updated", () => {
    const expr = computed(() => viewModel.triangle("foo"), { equals: () => false });
    const dispose = observe(expr, onChange);
    const triangle = viewModel.triangle("foo");
    model.color = "green";
    expect(triangle.color).toBe("green");
    expect(triangle.disposed).toBe(false);
    expect(onChange).toBeCalledTimes(1);
    dispose();
  });

  it("disposes when no longer observed", () => {
    const expr = computed(() => viewModel.triangle("foo"), { equals: () => false });
    const dispose = observe(expr, onChange);
    const triangle = viewModel.triangle("foo");
    dispose();
    expect(triangle.disposed).toBe(true);
  });
});
