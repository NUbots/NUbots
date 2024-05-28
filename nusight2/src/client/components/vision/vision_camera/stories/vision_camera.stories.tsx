import * as React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observable } from "mobx";
import { Observer, observer } from "mobx-react";
import * as THREE from "three";

import { SeededRandom } from "../../../../../shared/base/random/seeded_random";
import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import useInterval from "../../../../hooks/use-interval";
import { fakeCameraModel } from "../../../camera/stories/fake_camera_model";
import { RobotModel } from "../../../robot/model";
import { VisionRobotModel } from "../../model";
import { GreenHorizonModel } from "../green_horizon";
import { VisionCameraModel } from "../model";
import { VisionCameraView, VisionCameraViewProps } from "../view";
import { VisionCameraViewModel } from "../view_model";

type StoryComponent = React.FunctionComponent<
  Pick<VisionCameraViewProps, "objectFit" | "viewType" | "allowPanAndZoom">
>;

const meta: Meta<StoryComponent> = {
  title: "components/vision/VisionCamera",
  argTypes: {
    viewType: {
      control: "inline-radio",
      options: ["full", "thumbnail"],
    },
    objectFit: {
      control: "inline-radio",
      options: ["fill", "contain", "cover"],
    },
    allowPanAndZoom: {
      control: "boolean",
    },
  },
  args: {
    viewType: "full",
    objectFit: "fill",
    allowPanAndZoom: false,
  },
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="w-screen h-screen">{story()}</div>],
};

export default meta;

type Story = StoryObj<StoryComponent>;

const robotModelOpts = {
  address: "239.226.152.162",
  connected: true,
  enabled: true,
  id: "nuclearnet_client_1",
  name: "nuclearnet_client_1",
  port: 7447,
  type: "nuclearnet-peer",
} as const;

export const WithVisionRenderables: Story = {
  render: ({ objectFit, viewType, allowPanAndZoom }) => {
    const robot = VisionRobotModel.of(RobotModel.of(robotModelOpts));
    const box = observable<{ model: VisionCameraModel | undefined; viewModel: VisionCameraViewModel | undefined }>({
      model: undefined,
      viewModel: undefined,
    });

    fakeVisionCameraModel().then((model) => {
      box.model = model;
      box.viewModel = VisionCameraViewModel.of(box.model, robot);
    });

    return (
      <Observer>
        {() =>
          box.model && box.viewModel ? (
            <div className="flex flex-col h-screen w-screen">
              <div className="text-center py-2">
                Open the Storybook Controls to adjust view type, object fit, and allow pan and zoom.
              </div>
              <div className="h-full w-full">
                <VisionCameraView
                  model={box.model}
                  viewModel={box.viewModel}
                  robot={robot}
                  viewType={viewType}
                  objectFit={objectFit}
                  allowPanAndZoom={allowPanAndZoom}
                />
              </div>
            </div>
          ) : null
        }
      </Observer>
    );
  },
};

export const ObjectFitFillResizing: Story = {
  name: "Object Fit: fill with resizing",
  parameters: {
    // Hide the controls for this story
    controls: { exclude: /.*/ },
  },
  render: (_props) => {
    const robot = VisionRobotModel.of(RobotModel.of(robotModelOpts));
    const box = observable<{ model: VisionCameraModel | undefined; viewModel: VisionCameraViewModel | undefined }>({
      model: undefined,
      viewModel: undefined,
    });

    fakeVisionCameraModel().then((model) => {
      box.model = model;
      box.viewModel = VisionCameraViewModel.of(box.model, robot);
    });

    const width = observable.box(1);
    const random = SeededRandom.of("camera_view_fill");

    const Component = observer(() => {
      useInterval(() => width.set(random.float()), 1500);
      return (
        <div
          className="h-full"
          style={{
            width: `${width.get() * 85 + 15}%`,
            transition: "width 500ms ease",
          }}
        >
          {box.model && box.viewModel ? (
            <VisionCameraView objectFit="fill" model={box.model} viewModel={box.viewModel} robot={robot} />
          ) : null}
        </div>
      );
    });

    return <Component />;
  },
};

async function fakeVisionCameraModel(): Promise<VisionCameraModel> {
  const baseModel = await fakeCameraModel();

  const Hcw = baseModel.params.Hcw;
  const Hwc = Matrix4.fromThree(new THREE.Matrix4().copy(Hcw.toThree()).invert());

  const viewSize = Vector2.of(baseModel.image.width, baseModel.image.height);
  const focalLength = 193 / viewSize.x;

  return VisionCameraModel.of({
    ...baseModel,
    id: 0,
    greenHorizon: new GreenHorizonModel({
      horizon: [
        new Vector2(38, 258),
        new Vector2(248, 136),
        new Vector2(422, 164),
        new Vector2(539, 234),
        new Vector2(567, 333),
        new Vector2(445, 473),
        new Vector2(117, 422),
        new Vector2(38, 258),
      ].map((p) => screenToWorldRay(p, viewSize, focalLength, Hwc)),
      Hcw,
    }),
    balls: [
      {
        timestamp: 0,
        Hcw,
        cone: {
          axis: unprojectEquidistant(new Vector2(195, 303), viewSize, focalLength),
          radius: 0.994,
        },
        distance: 1,
        colour: new Vector4(1, 0.5, 0, 1),
      },
    ],
    goals: [
      {
        timestamp: 0,
        Hcw,
        side: "left",
        post: {
          top: unprojectEquidistant(new Vector2(63, 150), viewSize, focalLength),
          bottom: unprojectEquidistant(new Vector2(80, 218), viewSize, focalLength),
          distance: 4,
        },
      },
      {
        timestamp: 0,
        Hcw,
        side: "right",
        post: {
          top: unprojectEquidistant(new Vector2(197, 68), viewSize, focalLength),
          bottom: unprojectEquidistant(new Vector2(197, 152), viewSize, focalLength),
          distance: 4.5,
        },
      },
    ],
  });
}

function screenToWorldRay(screenPoint: Vector2, viewSize: Vector2, focalLength: number, Hwc: Matrix4) {
  const rFCc = unprojectEquidistant(screenPoint, viewSize, focalLength);
  const Rwc = new THREE.Matrix4().extractRotation(Hwc.toThree());
  return Vector3.fromThree(rFCc.toThree().applyMatrix4(Rwc)); // rFCw
}

function unprojectEquidistant(point: Vector2, viewSize: Vector2, focalLength: number): Vector3 {
  const p = new Vector2(viewSize.x / 2, viewSize.y / 2).subtract(point).divideScalar(viewSize.x);
  const r = p.length;
  const theta = r / focalLength;
  return new Vector3(
    Math.cos(theta),
    r !== 0 ? (Math.sin(theta) * p.x) / r : 0,
    r !== 0 ? (Math.sin(theta) * p.y) / r : 0,
  ).normalize();
}
