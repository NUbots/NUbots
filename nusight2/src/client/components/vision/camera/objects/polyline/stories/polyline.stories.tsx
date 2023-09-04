import * as React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observable } from "mobx";
import { Observer } from "mobx-react";
import { range } from '../../../../../../../shared/base/range'

import { CameraModel } from "../../../model";
import { fakeCameraModel } from "../../../stories/fake_camera_model";
import { PolylineView } from "../polyline";
import { Polyline } from "../polyline";
import { PolylinesView } from "../polyline";
import { Vector4 } from "../../../../../../../shared/math/vector4";
import { OrthographicCamera, ThreeFiber } from "../../../../../three/three_fiber";
import { Vector2 } from "../../../../../../../shared/math/vector2";

const meta: Meta<typeof PolylinesView> = {
  title: "components/Camera/objects/Polyline",
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="w-screen h-screen">{story()}</div>],
};

export default meta;

type Story = StoryObj<typeof PolylinesView>;

export const SinglePolyline: Story = {
  render: () => {
    const box = observable<{ camera: CameraModel | undefined; polyline: Polyline | undefined }>({
      camera: undefined,
      polyline: undefined,
    });

    // Create line data from random pixel positions
    fakeCameraModel().then((camera) => {
      box.camera = camera;
      box.polyline = {
        points: [
          [40, 280],
          [70, 225],
          [240, 140],
          [380, 150],
          [530, 220],
          [530, 390],
          [330, 460],
          [230, 430],
        ].map(([x, y]) => {
          const color = new Vector4(0, y / camera.image.height, x / camera.image.width, 1);
          return { pixel: new Vector2(x, y), color, size: 10 };
        }),
        autoClose: true,
        width: 5,
      };
    });

    return (
      <Observer>
        {() =>
          box.polyline && box.camera ? (
            <div className="w-full h-full">
              <ThreeFiber>
                <OrthographicCamera args={[-1, 1, 1, -1, 0, 1]} manual />
                {/* TODO(Annable): Add back in once vision is R3F */}
                {/*<ImageView image={box.camera.image} />*/}
                <PolylineView polyline={box.polyline} camera={box.camera} />
              </ThreeFiber>
            </div>
          ) : null
        }
      </Observer>
    );
  },
};

export const MultiPolyline: Story = {
  render: () => {
    const box = observable<{ camera: CameraModel | undefined; polylines: Polyline[] | undefined }>({
      camera: undefined,
      polylines: undefined,
    });

    // Create line data from random pixel positions
    fakeCameraModel().then((camera) => {
      box.camera = camera;
      box.polylines = [
        {
          points: [
            [40, 280],
            [70, 225],
            [240, 140],
            [380, 150],
            [530, 220],
            [530, 390],
            [330, 460],
            [230, 430],
          ].map(([x, y]) => {
            const color = new Vector4(0, y / camera.image.height, x / camera.image.width, 1);
            return { pixel: new Vector2(x, y), color, size: 10 };
          }),
          autoClose: true,
          width: 5,
        },
        {
          points: [
            [75, 210],
            [55, 90],
            [195, 65],
            [197, 150],
          ].map(([x, y]) => {
            const color = new Vector4(1, 1, 1, 0.7);
            return { pixel: new Vector2(x, y), color, size: 20 };
          }),
          width: 10,
        },
        {
          points: range(6).map((i) => {
            const color = new Vector4(1, 0.5, 0.0, 1);
            const pixel = new Vector2(Math.cos((2 * Math.PI * i) / 6), Math.sin((2 * Math.PI * i) / 6))
              .multiplyScalar(30)
              .add(new Vector2(196, 304));
            return { pixel, color, size: 8 };
          }),
          autoClose: true,
          width: 3,
        },
      ];
    });

    return (
      <Observer>
        {() =>
          box.polylines && box.camera ? (
            <div className="w-full h-full">
              <ThreeFiber>
                <OrthographicCamera args={[-1, 1, 1, -1, 0, 1]} manual />
                {/* TODO(Annable): Add back in once vision is R3F */}
                {/*<ImageView image={box.camera.image} />*/}
                <PolylinesView polylines={box.polylines} camera={box.camera} />
              </ThreeFiber>
            </div>
          ) : null
        }
      </Observer>
    );
  },
};
