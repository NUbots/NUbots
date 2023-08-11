import React from "react";
import { observer } from "mobx-react";

import { Transform } from "../../../shared/math/transform";
import { Appearance } from "../appearance/appearance";
import { BasicAppearance } from "../appearance/basic_appearance";
import { LineAppearance } from "../appearance/line_appearance";
import { Render2DEventHandlers } from "../event/event_handlers";
import { asSVGMouseEvent, asSVGWheelEvent } from "../event/mouse_event";
import { ArcGeometry } from "../geometry/arc_geometry";
import { ArrowGeometry } from "../geometry/arrow_geometry";
import { CircleGeometry } from "../geometry/circle_geometry";
import { LineGeometry } from "../geometry/line_geometry";
import { MarkerGeometry } from "../geometry/marker_geometry";
import { PathGeometry } from "../geometry/path_geometry";
import { PolygonGeometry } from "../geometry/polygon_geometry";
import { TextGeometry } from "../geometry/text_geometry";
import { Geometry } from "../object/geometry";
import { Shape } from "../object/shape";
import { SVGRendererTransforms } from "../svg_renderer";

import { Arc } from "./arc";
import { Arrow } from "./arrow";
import { Circle } from "./circle";
import { Line } from "./line";
import { Marker } from "./marker";
import { Path } from "./path";
import { Polygon } from "./polygon";
import { Text } from "./text";

export function toSvgEventHandlers(eventHandlers: Render2DEventHandlers, transforms: SVGRendererTransforms) {
  return {
    onMouseDown: asSVGMouseEvent(transforms, eventHandlers.onMouseDown),
    onMouseUp: asSVGMouseEvent(transforms, eventHandlers.onMouseUp),
    onMouseEnter: asSVGMouseEvent(transforms, eventHandlers.onMouseEnter),
    onMouseLeave: asSVGMouseEvent(transforms, eventHandlers.onMouseLeave),
    onMouseMove: asSVGMouseEvent(transforms, eventHandlers.onMouseMove),
    onClick: asSVGMouseEvent(transforms, eventHandlers.onClick),
    onContextMenu: asSVGMouseEvent(transforms, eventHandlers.onContextMenu),
    onWheel: asSVGWheelEvent(transforms, eventHandlers.onWheel),
  };
}

export function toSvgAppearance(appearance: Appearance) {
  if (appearance instanceof BasicAppearance) {
    return {
      ...(appearance.fill
        ? {
            fill: appearance.fill.color,
            fillOpacity: appearance.fill.alpha,
          }
        : {
            fill: "transparent",
          }),
      ...(appearance.stroke
        ? {
            strokeWidth: appearance.stroke.width,
            stroke: appearance.stroke.color,
            strokeOpacity: appearance.stroke.alpha,
            ...(appearance.stroke.nonScaling ? { vectorEffect: "non-scaling-stroke" } : {}),
          }
        : {
            stroke: "none",
          }),
      cursor: appearance.cursor,
    };
  } else if (appearance instanceof LineAppearance) {
    return {
      strokeLinecap: appearance.stroke.cap,
      strokeLinejoin: appearance.stroke.join,
      strokeDashoffset: appearance.stroke.dashOffset,
      strokeWidth: appearance.stroke.width,
      stroke: appearance.stroke.color,
      strokeOpacity: appearance.stroke.alpha,
      ...(appearance.stroke.nonScaling ? { vectorEffect: "non-scaling-stroke" } : {}),
      fill: "transparent",
    };
  } else {
    throw new Error(`Unsupported appearance type ${appearance}`);
  }
}

export function toSvgTransform(transform: Transform): string {
  const s = transform.scale;
  const r = (180.0 / Math.PI) * transform.rotate; // SVG rotations are in degrees
  const t = transform.translate;
  return `translate(${t.x}, ${t.y}) rotate(${r}) scale(${s.x}, ${s.y})`;
}

type Props = { obj: Shape<Geometry> };

export const ShapeView = observer(({ obj }: Props): JSX.Element => {
  if (obj.geometry instanceof ArcGeometry) {
    return <Arc model={obj as Shape<ArcGeometry>} />;
  } else if (obj.geometry instanceof ArrowGeometry) {
    return <Arrow model={obj as Shape<ArrowGeometry>} />;
  } else if (obj.geometry instanceof CircleGeometry) {
    return <Circle model={obj as Shape<CircleGeometry>} />;
  } else if (obj.geometry instanceof LineGeometry) {
    return <Line model={obj as Shape<LineGeometry>} />;
  } else if (obj.geometry instanceof MarkerGeometry) {
    return <Marker model={obj as Shape<MarkerGeometry>} />;
  } else if (obj.geometry instanceof PathGeometry) {
    return <Path model={obj as Shape<PathGeometry>} />;
  } else if (obj.geometry instanceof PolygonGeometry) {
    return <Polygon model={obj as Shape<PolygonGeometry>} />;
  } else if (obj.geometry instanceof TextGeometry) {
    return <Text model={obj as Shape<TextGeometry>} />;
  } else {
    throw new Error(`Unsupported geometry type: ${obj}`);
  }
});
