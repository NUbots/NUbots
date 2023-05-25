import { MouseEvent, WheelEvent } from "react";

import { Vector2 } from "../../../shared/math/vector2";
import { SVGRendererTransforms } from "../svg_renderer";

export interface Render2DMouseEvent {
  button: number;
  /** X Position of mouse relative to the Renderer view */
  x: number;
  /** Y Position of mouse relative to the Renderer view */
  y: number;
  /** X Position of mouse relative to the world */
  worldX: number;
  /** Y Position of mouse relative to the world */
  worldY: number;
  /** X Position of mouse relative to the current camera viewport */
  localX: number;
  /** Y Position of mouse relative to the current camera viewport */
  localY: number;
  ctrlKey: boolean;
  shiftKey: boolean;
  isPropagationStopped: () => boolean;
  stopPropagation: () => void;
  defaultPrevented: () => boolean;
  preventDefault: () => void;
}

export interface Render2DWheelEvent extends Render2DMouseEvent {
  deltaY: number;
}

/**
 * Takes a Render2D Mouse event callback and returns a function that can be assigned as the event for
 * an SVG element, by converting the React Mouse event to a Render2D Mouse event when it is triggered.
 *
 * This allows a shape's events to be bound to the SVG element, while still allowing the shape to use
 * the Render2D Mouse event interface.
 *
 * @param camera  Transform from svg coordinates to renderer world space
 * @param cb      Render2D Mouse event callback
 *
 * @returns       New callback wrapping original callback which can be assigned to an SVG element
 */
export const asSVGMouseEvent = (transforms: SVGRendererTransforms, cb?: (event: Render2DMouseEvent) => void) => {
  return (
    cb &&
    ((event: MouseEvent) => {
      cb(reactMouseEventTo2DRenderer(transforms, event));
    })
  );
};

export const asSVGWheelEvent = (transforms: SVGRendererTransforms, cb?: (event: Render2DWheelEvent) => void) => {
  return (
    cb &&
    ((event: WheelEvent) => {
      const renderEvent = {
        ...reactMouseEventTo2DRenderer(transforms, event),
        deltaY: event.deltaY,
      };
      cb(renderEvent);
    })
  );
};

/**
 * Convert React Mouse Event object to Render2D Mouse Event interface.
 *
 * @param camera  Transform from svg coordinates to renderer world space
 * @param event   React Mouse Event
 *
 * @returns       Render 2D Mouse Event
 */
export const reactMouseEventTo2DRenderer = (
  transforms: SVGRendererTransforms,
  event: MouseEvent,
): Render2DMouseEvent => {
  const { ctrlKey, shiftKey } = event;
  const { camera, world, getSVGOffset } = transforms;

  const pixel = Vector2.of(event.clientX, event.clientY).subtract(getSVGOffset());
  const worldPos = Vector2.of(pixel.x, pixel.y).transform(world.inverse());
  const localPos = Vector2.of(pixel.x, pixel.y).transform(camera.inverse());

  return {
    button: event.button,
    x: pixel.x,
    y: pixel.y,
    worldX: worldPos.x,
    worldY: worldPos.y,
    localX: localPos.x,
    localY: localPos.y,
    ctrlKey,
    shiftKey,
    isPropagationStopped: () => event.isPropagationStopped(), // Wrap in new function to keep target on original event
    stopPropagation: () => event.stopPropagation(),
    defaultPrevented: () => event.defaultPrevented,
    preventDefault: () => event.preventDefault(),
  };
};
