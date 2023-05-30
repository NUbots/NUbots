import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Transform } from "../../../shared/math/transform";
import { Vector2 } from "../../../shared/math/vector2";
import { BasicAppearance } from "../appearance/basic_appearance";
import { LineAppearance } from "../appearance/line_appearance";
import { ArcGeometry } from "../geometry/arc_geometry";
import { ArrowGeometry } from "../geometry/arrow_geometry";
import { CircleGeometry } from "../geometry/circle_geometry";
import { LineGeometry } from "../geometry/line_geometry";
import { MarkerGeometry } from "../geometry/marker_geometry";
import { PathGeometry } from "../geometry/path_geometry";
import { PolygonGeometry } from "../geometry/polygon_geometry";
import { TextGeometry } from "../geometry/text_geometry";
import { Group } from "../object/group";
import { Shape } from "../object/shape";
import { Renderer } from "../renderer";

type StoryComponent = React.FunctionComponent<{ renderEngine: "svg" | "canvas" }>;

const meta: Meta<StoryComponent> = {
  title: "components/Render2D",
  argTypes: {
    renderEngine: {
      control: "inline-radio",
      options: ["svg", "canvas"],
    },
  },
  args: {
    renderEngine: "svg",
  },
};

export default meta;

type Story = StoryObj<StoryComponent>;

const fillAppearance = BasicAppearance.of({
  fill: { color: "#FF0000", alpha: 1 },
});

const strokeAppearance = BasicAppearance.of({
  stroke: { color: "#000000", alpha: 1, nonScaling: true },
});

export const Arc: Story = {
  name: "Arc",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: ArcGeometry.of({
            origin: Vector2.of(-0.125, 0.125),
            radius: 0.125,
            startAngle: 0,
            endAngle: (3 * Math.PI) / 2,
            anticlockwise: true,
          }),
          appearance: strokeAppearance,
        }),
        Shape.of({
          geometry: ArcGeometry.of({
            origin: Vector2.of(0.125, -0.125),
            radius: 0.125,
            startAngle: Math.PI / 2,
            endAngle: Math.PI,
            anticlockwise: false,
          }),
          appearance: fillAppearance,
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Top-Left - Stroked Black 75% Arc w/ Gap Toward Centre</p>
        <p>Bottom-Right - Filled Red 75% Arc w/ Gap Toward Centre</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Arrow: Story = {
  name: "Arrow",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: ArrowGeometry.of({
            origin: Vector2.of(0.125, -0.125),
            direction: Vector2.of(1, -1),
            length: 0.125,
            headLength: 0.1,
            headWidth: 0.1,
            width: -0.05,
          }),
          appearance: fillAppearance,
        }),
        Shape.of({
          geometry: ArrowGeometry.of({
            origin: Vector2.of(-0.125, 0.125),
            direction: Vector2.of(-1, 1),
            length: 0.125,
            headLength: 0.1,
            headWidth: 0.1,
            width: -0.05,
          }),
          appearance: strokeAppearance,
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Top-Left - Top-Left Pointing Stroked Black Arrow</p>
        <p>Bottom-Right - Bottom-Right Pointing Filled Red Arrow</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Circle: Story = {
  name: "Circle",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: CircleGeometry.of({
            x: -0.25,
            y: 0.125,
            radius: 0.125,
          }),
          appearance: strokeAppearance,
        }),
        Shape.of({
          geometry: CircleGeometry.of({
            x: 0.25,
            y: -0.125,
            radius: 0.125,
          }),
          appearance: fillAppearance,
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Top-Left - Stroked Black Circle</p>
        <p>Bottom-Right - Filled Red Circle</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Line: Story = {
  name: "Line",
  render: ({ renderEngine }) => {
    const lineAppearance = LineAppearance.of({ stroke: { color: "#000000", nonScaling: true, alpha: 1 } });
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: LineGeometry.of({
            origin: Vector2.of(-0.25, -0.25),
            target: Vector2.of(-0.25, 0.25),
          }),
          appearance: lineAppearance,
        }),
        Shape.of({
          geometry: LineGeometry.of({
            origin: Vector2.of(-0.25, 0.0),
            target: Vector2.of(0.25, 0.0),
          }),
          appearance: lineAppearance,
        }),
        Shape.of({
          geometry: LineGeometry.of({
            origin: Vector2.of(0.25, 0.0),
            target: Vector2.of(0.25, -0.25),
          }),
          appearance: lineAppearance,
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Lower case &#39;h&#39; shape made with Lines</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Marker: Story = {
  name: "Marker",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: MarkerGeometry.of({
            x: -0.25,
            y: 0.25,
            radius: 0.125,
            heading: Vector2.of(1, -1),
          }),
          appearance: strokeAppearance,
        }),
        Shape.of({
          geometry: MarkerGeometry.of({
            x: 0.25,
            y: -0.25,
            radius: 0.125,
            heading: Vector2.of(-1, 1),
          }),
          appearance: fillAppearance,
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Top-Left - Bottom-Right-Pointing Stroked Black Marker</p>
        <p>Bottom-Right - Top-Left-Pointing Filled Red Marker</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Path: Story = {
  name: "Path",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: PathGeometry.of([
            Vector2.of(-0.25, -0.25),
            Vector2.of(-0.25, 0.25),
            Vector2.of(0.25, 0.25),
            Vector2.of(0.25, 0.0),
            Vector2.of(-0.25, 0.0),
            Vector2.of(0.25, -0.25),
          ]),
          appearance: LineAppearance.of({ stroke: { color: "#000000", nonScaling: true, alpha: 1 } }),
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Upper case &#39;R&#39; shape made with a Path</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Polygon: Story = {
  name: "Polygon",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: PolygonGeometry.of([
            Vector2.of(-0.25, 0.125),
            Vector2.of(-0.125, 0.125),
            Vector2.of(-0.125, 0.0),
            Vector2.of(-0.25, 0.0),
          ]),
          appearance: strokeAppearance,
        }),
        Shape.of({
          geometry: PolygonGeometry.of([
            Vector2.of(0.125, 0.0),
            Vector2.of(0.25, 0.0),
            Vector2.of(0.25, -0.125),
            Vector2.of(0.125, -0.125),
          ]),
          appearance: fillAppearance,
        }),
      ],
    });
    const camera = Transform.of({});
    return (
      <div>
        <p>Top-Left - Stroked Black Rectangle</p>
        <p>Bottom-Right - Filled Red Rectangle</p>
        <Renderer engine={renderEngine} scene={scene} camera={camera} aspectRatio={1} />
      </div>
    );
  },
};

export const Text: Story = {
  name: "Text",
  render: ({ renderEngine }) => {
    const scene = Group.of({
      children: [
        Shape.of({
          geometry: TextGeometry.of({
            text: "Medium Size Font, Align Middle",
            fontSize: "medium",
            x: 0,
            y: 0,
            textAlign: "middle",
            textBaseline: "middle",
            worldAlignment: true,
            worldScale: true,
          }),
          appearance: fillAppearance,
        }),
        Shape.of({
          geometry: TextGeometry.of({
            text: "Small Size Font, Align End",
            fontSize: "small",
            x: 0,
            y: 0.25,
            textAlign: "end",
            textBaseline: "middle",
            worldAlignment: false,
            worldScale: true,
          }),
          appearance: fillAppearance,
        }),
        Shape.of({
          geometry: TextGeometry.of({
            text: "Large Size Font, Align Start",
            fontSize: "large",
            x: 0,
            y: -0.25,
            textAlign: "start",
            textBaseline: "middle",
            worldAlignment: true,
            worldScale: true,
          }),
          appearance: fillAppearance,
        }),
      ],
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
