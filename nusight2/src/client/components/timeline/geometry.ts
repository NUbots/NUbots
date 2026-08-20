import { CSSProperties } from "react";
import { BasicAppearance } from "@client/render2d/appearance/basic_appearance";
import { LineAppearance } from "@client/render2d/appearance/line_appearance";
import { Render2DEventHandlers } from "@client/render2d/event/event_handlers";
import { CircleGeometry } from "@client/render2d/geometry/circle_geometry";
import { LineGeometry } from "@client/render2d/geometry/line_geometry";
import { PolygonGeometry } from "@client/render2d/geometry/polygon_geometry";
import { RectangleGeometry } from "@client/render2d/geometry/rectangle_geometry";
import { TextGeometry } from "@client/render2d/geometry/text_geometry";
import { Geometry } from "@client/render2d/object/geometry";
import { Group } from "@client/render2d/object/group";
import { Shape, ShapeOpts } from "@client/render2d/object/shape";
import { Transform } from "@shared/math/transform";
import { Vector2 } from "@shared/math/vector2";
import { Property } from "csstype";

interface CircleOpts {
  x: number;
  y: number;
  radius: number;
  color?: string;
  cursor?: CSSProperties["cursor"];
  eventHandlers?: Render2DEventHandlers;
}

export function circle(opts: CircleOpts) {
  const { x, y, radius, color = "#000000", cursor, eventHandlers } = opts;
  return Shape.of({
    geometry: CircleGeometry.of({ x, y, radius }),
    appearance: BasicAppearance.of({ fill: { color }, cursor }),
    eventHandlers,
  });
}

interface TextOpts {
  text: string;
  x: number;
  y: number;
  color: string;
  /** Alpha value from 0 to 1 */
  alpha?: number;
  /** Size of the text with a unit (e.g "10px") */
  size?: string;
  align?: TextGeometry["textAlign"];
  baseline?: TextGeometry["textBaseline"];
  /** @deprecated pointerEvents is not supported in NUbots render2d; prop is accepted but ignored */
  pointerEvents?: ShapeOpts<Geometry> extends { pointerEvents?: infer P } ? P : string;
}

/** Shorthand for creating a text geometry */
export function text(opts: TextOpts) {
  const { text, x, y, color, alpha, size = "15px", align = "middle", baseline = "bottom" } = opts;
  return Shape.of({
    geometry: TextGeometry.of({
      text,
      x,
      y,
      textAlign: align,
      textBaseline: baseline,
      worldScale: true,
      fontSize: size,
    }),
    appearance: BasicAppearance.of({ fill: { color, alpha } }),
  });
}

interface LineOpts {
  x: number;
  x2?: number;
  y: number;
  y2?: number;
  color?: string;
  width?: number;
  alpha?: number;
  cursor?: Property.Cursor;
  cap?: "butt" | "round" | "square";
  ignorePointerEvents?: boolean;
  eventHandlers?: Render2DEventHandlers;
}

/** Shorthand for creating a line geometry */
export function line(opts: LineOpts) {
  const {
    x,
    x2 = x,
    y,
    y2 = y,
    color = "#000000",
    width = 1,
    alpha = 1,
    cap = "butt",
    ignorePointerEvents,
    eventHandlers,
  } = opts;
  return Shape.of({
    geometry: LineGeometry.of({
      origin: Vector2.of(x, y),
      target: Vector2.of(x2, y2),
    }),
    appearance: LineAppearance.of({ stroke: { cap, color, alpha, width, nonScaling: true } }),
    eventHandlers: ignorePointerEvents ? undefined : eventHandlers,
  });
}

interface RectangleOpts {
  x?: number;
  y?: number;
  width: number;
  height: number;
  color: string;
  border?: string;
  borderWidth?: number;
  /** @deprecated dashArray not supported in NUbots BasicAppearance; prop accepted but ignored */
  borderDashArray?: string;
  /**
   * If set to `full`, will use the larger of width or height to calculate the border radius, for a "full"
   * pill shaped rounding.
   */
  borderRadius?: number | "full";
  cursor?: CSSProperties["cursor"];
  eventHandlers?: Render2DEventHandlers;
  /** @deprecated pointerEvents is not supported in NUbots render2d; prop is accepted but ignored */
  pointerEvents?: CSSProperties["pointerEvents"];
  alpha?: number;
}

/** Shorthand for drawing a rectangle */
export function rectangle(opts: RectangleOpts) {
  const {
    x = 0,
    y = 0,
    width,
    height,
    color,
    border = color,
    borderWidth,
    borderRadius,
    cursor,
    eventHandlers,
    alpha,
  } = opts;
  return Shape.of({
    geometry: RectangleGeometry.of(x, y, width, height, borderRadius),
    appearance: BasicAppearance.of({
      fill: { color, alpha },
      stroke:
        border !== "transparent"
          ? {
              color: border,
              width: borderWidth,
              nonScaling: true,
            }
          : undefined,
      cursor,
    }),
    eventHandlers,
  });
}

interface MarkerOpts {
  color: string;
  ignorePointerEvents?: boolean;
  eventHandlers?: Render2DEventHandlers;
}

/** Shorthand for the geometry of the time markers */
export function markerShape({ color, ignorePointerEvents, eventHandlers }: MarkerOpts) {
  return Shape.of({
    geometry: PolygonGeometry.of([
      Vector2.of(-7, -18),
      Vector2.of(7, -18),
      Vector2.of(7, -6),
      Vector2.of(0, 0),
      Vector2.of(-7, -6),
    ]),
    appearance: BasicAppearance.of({ fill: { color }, cursor: "pointer" }),
    eventHandlers: ignorePointerEvents ? undefined : eventHandlers,
  });
}

interface FlagOpts {
  x: number;
  y: number;
  color: string;
  outline?: string;
  direction: "left" | "right";
  eventHandlers?: Render2DEventHandlers;
}

export function flag({ x, y, color, outline, direction, eventHandlers }: FlagOpts) {
  return Group.of({
    children: [
      Shape.of({
        geometry: PolygonGeometry.of([
          Vector2.of(16, -15),
          Vector2.of(0, -15),
          Vector2.of(0, 0),
          Vector2.of(8, -5),
          Vector2.of(16, -5),
        ]),
        appearance: BasicAppearance.of({
          fill: { color },
          stroke: outline ? { color: outline } : undefined,
          cursor: "pointer",
        }),
        eventHandlers,
      }),
    ],
    transform: Transform.of({ translate: { x, y }, scale: { x: direction === "left" ? -1 : 1, y: 1 } }),
  });
}

interface LabelOpts {
  text: string;
  color: string;
  textColor: string;
}

/** Shorthand for geometry containing text over a rectangular background */
export function label(opts: LabelOpts): [Shape<RectangleGeometry>, Shape<TextGeometry>] {
  const y = -49;
  const width = 100;
  const height = 28;
  return [
    rectangle({ x: -width / 2, y, width, height, color: opts.color }),
    text({
      x: 0,
      y: y + height / 2,
      text: opts.text,
      color: opts.textColor,
      baseline: "middle",
    }),
  ];
}
