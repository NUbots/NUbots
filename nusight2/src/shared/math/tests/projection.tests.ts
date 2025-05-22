import { describe, expect, it } from "vitest";

import { project, Projection, unproject } from "../projection";
import { Vector2 } from "../vector2";
import { Vector3 } from "../vector3";

interface Lens {
  projection: Projection;
  f: number;
  fov: number;
  center: Vector2;
  k: Vector2;
}

const lenses = [
  // RGB10FOV
  {
    lens: {
      projection: Projection.RECTILINEAR,
      f: 5.8584539601245105,
      fov: 0.26203861879903206,
      center: new Vector2(0.12794484533176642, 0.030038261491061286),
      k: new Vector2(-0.005886046707870895, 0.0114287770512325),
    },
    dimensions: new Vector2(2448, 2048),
  },

  // RGB40FOV
  {
    lens: {
      projection: Projection.RECTILINEAR,
      f: 1.3805811312056515,
      fov: 0.9151210790561236,
      center: new Vector2(0.010797947122843954, -0.015031597322440565),
      k: new Vector2(0.03190653516072612, 0.0028458609077313024),
    },
    dimensions: new Vector2(2448, 2048),
  },

  // Thermal10FOV
  {
    lens: {
      projection: Projection.RECTILINEAR,
      f: 4.5221705474682246,
      fov: 0.31532484150061746,
      center: new Vector2(-0.02002094012325142, 0.12188789956869996),
      k: new Vector2(-0.015042396479241037, 0.02748844155080153),
    },
    dimensions: new Vector2(640, 480),
  },

  // Thermal40FOV
  {
    lens: {
      projection: Projection.RECTILINEAR,
      f: 1.358110842857084,
      fov: 0.956773110060132,
      center: new Vector2(-0.01319166248464335, 0.050925771003305816),
      k: new Vector2(0.11763344353568171, 0.018597493777086235),
    },
    dimensions: new Vector2(640, 480),
  },

  // CenterWide
  {
    lens: {
      projection: Projection.EQUIDISTANT,
      f: 0.31157748062787916,
      fov: Math.PI,
      center: new Vector2(-0.025244984530361225, 0.0002682050140004796),
      k: new Vector2(0.21100008102840612, 0.043052818523785126),
    },
    dimensions: new Vector2(1920, 1200),
  },
];

function runRoundTrip(lens: Lens, dimensions: Vector2, normalised: boolean = true) {
  const dims = normalised ? dimensions.divideScalar(dimensions.x) : dimensions;
  const center = new Vector2(lens.center.x, lens.center.y);
  const k = new Vector2(lens.k.x, lens.k.y);

  // Pick random direction for our ray
  const theta = (Math.random() - 1) * lens.fov * 0.5;
  let phi = Math.acos(2 * Math.random() - 1);

  // Map the direction to inside the camera's fov
  if (lens.fov < Math.PI) {
    const range_start = Math.PI * 0.5 - lens.fov * 0.5;
    const range_end = Math.PI * 0.5 + lens.fov * 0.5;
    phi = ((range_end - range_start) / Math.PI) * phi + range_start;
  }

  // Project and unproject to get 2 rays and 2 pixel positions
  const ray0 = new Vector3(Math.cos(theta) * Math.sin(phi), Math.sin(theta) * Math.sin(phi), Math.cos(phi));
  const px0 = project(ray0, lens.projection, lens.f, center, k, dims);
  const ray1 = unproject(px0, lens.projection, lens.f, center, k, dims);
  const px1 = project(ray1, lens.projection, lens.f, center, k, dims);

  // The pair of pixel positions and pair of rays should be the same value
  expect(px0.x).toBeCloseTo(px1.x);
  expect(px0.y).toBeCloseTo(px1.y);
  expect(ray0.x).toBeCloseTo(ray1.x);
  expect(ray0.y).toBeCloseTo(ray1.y);
  expect(ray0.z).toBeCloseTo(ray1.z);
}

describe("Projection", () => {
  describe("should be the same value in round trip", () => {
    it("when normalised using known lens parameters", () => {
      for (const { lens, dimensions } of lenses) {
        runRoundTrip(lens, dimensions, true);
      }
    });
    it("when unnormalised using known lens parameters", () => {
      for (const { lens, dimensions } of lenses) {
        runRoundTrip(lens, dimensions, false);
      }
    });
  });
});
