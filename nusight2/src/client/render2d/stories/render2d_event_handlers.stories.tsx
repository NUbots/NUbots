import React from "react";
import { action as storyAction } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";
import { action, computed, observable } from "mobx";
import { Observer } from "mobx-react";

import { Transform } from "../../../shared/math/transform";
import { Vector2 } from "../../../shared/math/vector2";
import { BasicAppearance } from "../appearance/basic_appearance";
import { Render2DEventHandlers } from "../event/event_handlers";
import { ArcGeometry } from "../geometry/arc_geometry";
import { PolygonGeometry } from "../geometry/polygon_geometry";
import { Geometry } from "../object/geometry";
import { Group } from "../object/group";
import { Shape } from "../object/shape";
import { Renderer } from "../renderer";

const meta: Meta<typeof Renderer> = {
  title: "components/render2d",
  component: Renderer,
};

export default meta;

enum ShapeColor {
  DEFAULT = "#FF0000",
  HOVERED = "#0000FF",
  HELD = "#00FF00",
}

interface ShapeData {
  geometry: Geometry;
  color: string;
  name: string;
  position: Vector2;
  rotation: number;
}

class ShapeModel {
  @observable mouseDown = false;
  @observable lastMouse?: Vector2;
  @observable cameraPos = Vector2.of(0, 0);
  @observable cameraScale = Vector2.of(1, 1);
  @observable heldShape?: ShapeData;

  @observable
  shapes: ShapeData[] = [
    {
      geometry: ArcGeometry.of({
        origin: Vector2.of(),
        radius: 0.075,
        startAngle: 0,
        endAngle: Math.PI / 2,
      }),
      color: ShapeColor.DEFAULT,
      name: "Arc",
      position: Vector2.of(-0.25, 0.2),
      rotation: 0,
    },
    {
      geometry: PolygonGeometry.of([Vector2.of(-0.075, 0.075), Vector2.of(0.075, -0.075), Vector2.of(-0.075, -0.075)]),
      color: ShapeColor.DEFAULT,
      name: "Polygon 1",
      position: Vector2.of(-0.25, -0.15),
      rotation: 0,
    },
    {
      geometry: PolygonGeometry.of([
        Vector2.of(-0.075, 0.075),
        Vector2.of(0, 0.15),
        Vector2.of(0.075, 0.075),
        Vector2.of(0.075, -0.075),
        Vector2.of(-0.075, -0.075),
      ]),
      color: ShapeColor.DEFAULT,
      name: "Polygon 2",
      position: Vector2.of(0.125, -0.125),
      rotation: 0,
    },
  ];

  @computed
  get scene(): Group {
    return Group.of({
      children: this.shapes.map((shape) => {
        const color = shape === this.heldShape ? ShapeColor.HELD : shape.color;
        return Group.of({
          children: [
            Shape.of({
              geometry: shape.geometry,
              appearance: BasicAppearance.of({
                fill: { color: color, alpha: 1 },
                cursor: this.heldShape === shape ? "grabbing" : "grab",
              }),
              eventHandlers: {
                onMouseEnter: action(() => {
                  shape.color = ShapeColor.HOVERED;
                }),
                onMouseLeave: action(() => {
                  shape.color = ShapeColor.DEFAULT;
                }),
                onMouseDown: action((e) => {
                  if (e.button === 0) {
                    this.heldShape = shape;
                  }
                }),
                onWheel: action((e) => {
                  shape.rotation += Math.sign(e.deltaY) * 0.2;
                  e.stopPropagation();
                }),
                onClick: action((e) => {
                  storyAction(`Shape Clicked: ${shape.name}`)(e);
                  e.stopPropagation();
                }),
                onContextMenu: action(() => {
                  shape.rotation += Math.PI;
                }),
              },
            }),
          ],
          transform: Transform.of({ translate: shape.position, rotate: shape.rotation }),
        });
      }),
    });
  }

  @computed
  get camera(): Transform {
    return Transform.of({
      translate: this.cameraPos,
      scale: this.cameraScale,
    });
  }

  get events(): Render2DEventHandlers {
    return {
      onClick: action((e) => {
        storyAction(`No Shape Clicked`)(e);
        e.stopPropagation();
      }),
      onMouseDown: action(() => {
        this.mouseDown = true;
      }),
      onMouseUp: action(() => {
        this.mouseDown = false;
        this.heldShape = undefined;
      }),
      onMouseMove: action((e) => {
        if (this.lastMouse && this.mouseDown && this.heldShape) {
          const movement = Vector2.of(e.localX, e.localY).subtract(this.lastMouse);
          this.heldShape.position = this.heldShape.position.add(movement.multiplyScalar(this.cameraScale.x));
        } else if (this.lastMouse && this.mouseDown) {
          const movement = Vector2.of(e.localX, e.localY).subtract(this.lastMouse);
          this.cameraPos = this.cameraPos.subtract(movement.multiplyScalar(this.cameraScale.x));
        }
        this.lastMouse = Vector2.of(e.localX, e.localY);
      }),
      onWheel: action((e) => {
        const scale = 1 + Math.sign(e.deltaY) * 0.25;
        this.cameraScale = this.cameraScale.multiplyScalar(scale);
      }),
      onContextMenu: action((e) => {
        e.preventDefault();
      }),
    };
  }
}

export const RendererEvents: StoryObj<typeof Renderer> = {
  name: "Event Handlers",
  render: () => {
    const shapeModel = new ShapeModel();
    return (
      <Observer>
        {() => (
          <div>
            <div>Controls:</div>
            <ul>
              <li> - Click and drag background to pan the camera</li>
              <li> - Use scroll wheel on the background to zoom in/out</li>
              <li> - Click and drag shapes to move them</li>
              <li> - Use scroll wheel on shapes to rotate them</li>
            </ul>
            <Renderer
              engine="svg"
              scene={shapeModel.scene}
              camera={shapeModel.camera}
              eventHandlers={shapeModel.events}
              aspectRatio={1}
            />
          </div>
        )}
      </Observer>
    );
  },
};
