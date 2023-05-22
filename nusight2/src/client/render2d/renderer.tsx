import React from "react";

import { Transform } from "../../shared/math/transform";

import { CanvasRenderer } from "./canvas_renderer";
import { Render2DEventHandlers } from "./event/event_handlers";
import { Group } from "./object/group";
import { SVGRenderer } from "./svg_renderer";

export type RendererProps = {
  className?: string;
  scene: Group;
  camera: Transform;
  eventHandlers?: Render2DEventHandlers;
  aspectRatio?: number;
  onResize?: (width: number, height: number) => void;
};

export const Renderer = (props: RendererProps & { engine?: "svg" | "canvas" }): JSX.Element => {
  switch (props.engine) {
    case undefined:
    case "svg":
      return <SVGRenderer {...props} />;
    case "canvas":
      return <CanvasRenderer {...props} />;
    default:
      throw new Error(`Unknown rendering engine ${props.engine}`);
  }
};
