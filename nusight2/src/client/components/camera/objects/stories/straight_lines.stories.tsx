import * as React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { computed, observable } from "mobx";
import { Observer } from "mobx-react";

import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraModel } from "../../model";
import { fakeCameraModel } from "../../stories/fake_camera_model";
import { CameraView, Renderable } from "../../view";
import { CameraViewModel } from "../../view_model";
import { LineGroupOpts, StraightLines } from "../straight_lines/straight_lines";

const meta: Meta<typeof StraightLines> = {
  title: "components/Camera/objects/Straight Line Renderer",
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="w-screen h-screen">{story()}</div>],
};

export default meta;

type Story = StoryObj<typeof StraightLines>;

function randomPixel(dims: { width: number; height: number }) {
  return Vector2.of(Math.random() * dims.width, Math.random() * dims.height);
}

function randomRay() {
  return Vector3.of(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
}

function randomColor() {
  return Vector4.of(Math.random(), Math.random(), Math.random(), 1);
}

/** View Model with a straight line renderer to add lines to the scene */
class StraightLineViewModel extends CameraViewModel {
  constructor(protected model: CameraModel, private lineData: LineGroupOpts) {
    super(model);
  }

  @computed
  get straightLines() {
    return new StraightLines(this.canvas, this.model.params, this.model.image);
  }

  protected getRenderables(): Renderable[] {
    return this.straightLines.lineGroup(this.lineData);
  }
}

export const StraightLineRendererRays: Story = {
  name: "Rays",
  render: () => {
    const box = observable<{ viewModel: StraightLineViewModel | undefined }>({
      viewModel: undefined,
    });

    // Create line data from rays in random directions
    fakeCameraModel().then((model) => {
      const loopCount = 4;
      const joinCount = 3;
      const lineData: LineGroupOpts = {
        type: "Ray",
        lines: new Array(loopCount).fill(0).map(() => ({
          type: "Ray",
          joins: new Array(joinCount).fill(0).map(() => ({ ray: randomRay(), color: randomColor(), size: 10 })),
          loop: true,
          width: 5,
        })),
      };

      box.viewModel = new StraightLineViewModel(model, lineData);
    });

    return (
      <Observer>
        {() =>
          box.viewModel ? (
            <div className="w-full h-full">
              <CameraView viewModel={box.viewModel} viewType="full" objectFit="fill" allowPanAndZoom={true} />
            </div>
          ) : null
        }
      </Observer>
    );
  },
};

export const StraightLineRendererPixels: Story = {
  name: "Pixels",
  render: () => {
    const box = observable<{ viewModel: StraightLineViewModel | undefined }>({
      viewModel: undefined,
    });

    // Create line data from random pixel positions
    fakeCameraModel().then((model) => {
      const loopCount = 4;
      const joinCount = 3;
      const lineData: LineGroupOpts = {
        type: "Pixel",
        lines: new Array(loopCount).fill(0).map(() => ({
          type: "Pixel",
          joins: new Array(joinCount)
            .fill(0)
            .map(() => ({ pixel: randomPixel(model.image), color: randomColor(), size: 10 })),
          loop: true,
          width: 5,
        })),
      };

      box.viewModel = new StraightLineViewModel(model, lineData);
    });

    return (
      <Observer>
        {() =>
          box.viewModel ? (
            <div className="w-full h-full">
              <CameraView viewModel={box.viewModel} viewType="full" objectFit="fill" allowPanAndZoom={true} />
            </div>
          ) : null
        }
      </Observer>
    );
  },
};
