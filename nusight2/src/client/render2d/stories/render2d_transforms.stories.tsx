import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Transform } from "../../../shared/math/transform";
import { Vector2 } from "../../../shared/math/vector2";
import { BasicAppearance } from "../appearance/basic_appearance";
import { CircleGeometry } from "../geometry/circle_geometry";
import { PolygonGeometry } from "../geometry/polygon_geometry";
import { TextGeometry } from "../geometry/text_geometry";
import { Group } from "../object/group";
import { Shape } from "../object/shape";
import { Renderer } from "../renderer";

type StoryComponent = React.FunctionComponent<{
  renderEngine: "svg" | "canvas";
  transformTarget: "Camera" | "Group";
  translation: number;
  rotation: number;
  scale: number;
}>;

const meta: Meta<StoryComponent> = {
  title: "components/Render2D/Transforms",
  argTypes: {
    renderEngine: {
      control: "inline-radio",
      options: ["svg", "canvas"],
    },
    transformTarget: {
      control: "inline-radio",
      options: ["Camera", "Group"],
    },
    translation: {
      control: {
        type: "range",
        min: -0.5,
        max: 0.5,
        step: 0.01,
        disableDebounce: true,
      },
    },
    rotation: {
      control: {
        type: "range",
        min: -3.14,
        max: 3.14,
        step: 0.01,
        disableDebounce: true,
      },
    },
    scale: {
      control: {
        type: "range",
        min: 0.2,
        max: 2,
        step: 0.1,
        disableDebounce: true,
      },
    },
  },
  args: {
    renderEngine: "svg",
    transformTarget: "Camera",
    rotation: 0,
    translation: 0,
    scale: 1,
  },
};

export default meta;

type Story = StoryObj<StoryComponent>;

const fillAppearance = BasicAppearance.of({
  fill: { color: "#FF0000", alpha: 1 },
});

export const CameraVsGroup: Story = {
  name: "Camera vs Group",
  render: ({ renderEngine, transformTarget, translation, rotation, scale }) => {
    const squareGeometry = PolygonGeometry.of([
      Vector2.of(-0.125, 0.25),
      Vector2.of(0.25, 0.25),
      Vector2.of(0.125, -0.25),
      Vector2.of(-0.125, -0.25),
    ]);

    const circleGeometry = CircleGeometry.of({
      radius: 0.125,
      x: -0.25,
      y: 0.125,
    });

    const transform = Transform.of({
      rotate: rotation,
      translate: Vector2.of(translation, translation),
      scale: Vector2.of(scale, scale),
    });

    const camera = transformTarget == "Camera" ? transform : Transform.of({});

    const polygon = Group.of({
      children: [Shape.of({ geometry: squareGeometry, appearance: fillAppearance })],
      transform: transformTarget == "Group" ? transform : Transform.of({}),
    });

    const scene = Group.of({
      children: [
        Shape.of({ geometry: circleGeometry, appearance: BasicAppearance.of({ fill: { color: "#FFFF00" } }) }),
        polygon,
      ],
    });

    return (
      <div>
        <p>Transform being applied to the camera vs a group. </p>
        <p>
          When applied to the group, only the red polygon should be affected. When applied to the camera, the transform
          should affect ALL shapes AND the transform should be be inverted.
        </p>
        <br />
        <p>Transforms Target: {transformTarget}.</p>
        <p>
          Translation: {transform.translate.x.toFixed(3)}, {transform.translate.y.toFixed(3)}
        </p>
        <p>Rotation: {transform.rotate.toFixed(3)}</p>
        <p>
          Scale: {transform.scale.x.toFixed(3)}, {transform.scale.y.toFixed(3)}
        </p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Nested: Story = {
  name: "Nested",
  render: ({ renderEngine, translation, rotation, scale }) => {
    const squareGeometry = PolygonGeometry.of([
      Vector2.of(-0.25, 0.25),
      Vector2.of(0.25, 0.25),
      Vector2.of(0.25, -0.25),
      Vector2.of(-0.25, -0.25),
    ]);

    const transform = Transform.of({
      rotate: rotation,
      translate: Vector2.of(translation, translation),
      scale: Vector2.of(scale, scale),
    });

    const scene = Group.of({
      transform: transform,
      children: [
        Shape.of({ geometry: squareGeometry, appearance: fillAppearance }),
        Group.of({
          transform: transform,
          children: [
            Shape.of({
              geometry: squareGeometry,
              appearance: BasicAppearance.of({
                fill: { color: "#FFFF00", alpha: 1 },
              }),
            }),
          ],
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Two Rectangles with the same geometry.</p>
        <p>The transform contains a translation, rotation, and scale.</p>
        <p>The transform is applied once to the Red Rectangle and twice to the Yellow.</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Text: Story = {
  name: "Text",
  render: ({ renderEngine, rotation }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: TextGeometry.of({
            text: "Align Middle, Rotating",
            x: 0.25,
            y: 0.25,
            textAlign: "middle",
            worldAlignment: false,
            worldScale: true,
          }),
          appearance: fillAppearance,
        }),
        Shape.of({
          geometry: TextGeometry.of({
            text: "Align End, Rotating",
            fontSize: "small",
            x: -0.25,
            y: 0.25,
            textAlign: "end",
            worldAlignment: false,
            worldScale: true,
          }),
          appearance: fillAppearance,
        }),
        Shape.of({
          geometry: TextGeometry.of({
            text: "Align Middle. View Aligned",
            x: -0.25,
            y: -0.25,
            textAlign: "start",
            worldAlignment: true,
            worldScale: true,
          }),
          appearance: fillAppearance,
        }),
      ],
      transform: Transform.of({
        rotate: rotation,
      }),
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Text Shapes each with a different y position, font size, and alignment</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};
