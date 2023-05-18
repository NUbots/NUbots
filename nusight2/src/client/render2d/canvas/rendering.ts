import { Transform } from "../../../shared/math/transform";
import { Appearance } from "../appearance/appearance";
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
import { Geometry } from "../object/geometry";
import { Group } from "../object/group";
import { Shape } from "../object/shape";

import { renderArc } from "./arc";
import { renderArrow } from "./arrow";
import { renderCircle } from "./circle";
import { renderLine } from "./line";
import { renderMarker } from "./marker";
import { renderPath } from "./path";
import { renderPolygon } from "./polygon";
import { renderText } from "./text";

export function renderObject2d(ctx: CanvasRenderingContext2D, obj: Group | Shape<Geometry>, world: Transform) {
  if (obj instanceof Group) {
    // Cumulative transform of the camera + parent groups
    const objWorld = world.then(obj.transform);
    for (const o of obj.children) {
      ctx.save();
      renderObject2d(ctx, o, objWorld);
      ctx.restore();
    }
  } else if (obj instanceof Shape && obj.geometry instanceof TextGeometry) {
    // Special case for text drawing, since it cannot be drawn in
    // ctx.beginPath() mode.
    applyTransform(ctx, world);
    renderText(ctx, obj as Shape<TextGeometry>, world);
    ctx.resetTransform();
  } else if (obj instanceof Shape) {
    applyTransform(ctx, world);

    if (obj.geometry instanceof ArcGeometry) {
      renderArc(ctx, obj as Shape<ArcGeometry>);
    } else if (obj.geometry instanceof ArrowGeometry) {
      renderArrow(ctx, obj as Shape<ArrowGeometry>);
    } else if (obj.geometry instanceof CircleGeometry) {
      renderCircle(ctx, obj as Shape<CircleGeometry>);
    } else if (obj.geometry instanceof LineGeometry) {
      renderLine(ctx, obj as Shape<LineGeometry>);
    } else if (obj.geometry instanceof MarkerGeometry) {
      renderMarker(ctx, obj as Shape<MarkerGeometry>);
    } else if (obj.geometry instanceof PathGeometry) {
      renderPath(ctx, obj as Shape<PathGeometry>);
    } else if (obj.geometry instanceof PolygonGeometry) {
      renderPolygon(ctx, obj as Shape<PolygonGeometry>);
    } else {
      throw new Error(`Unsupported geometry type: ${obj.geometry}`);
    }

    // Line width is affected by the scaling so we reset the transform
    // and only apply the dpi scaling.
    ctx.resetTransform();
    ctx.scale(devicePixelRatio, devicePixelRatio);

    applyAppearance(ctx, obj.appearance);
    if (obj.appearance.stroke) {
      ctx.stroke();
    }
    if (obj.appearance instanceof BasicAppearance && obj.appearance.fill) {
      ctx.fill();
    }
  } else {
    throw new Error(`Unsupported Object2d type: ${obj}`);
  }
}

export function applyTransform(ctx: CanvasRenderingContext2D, transform: Transform): void {
  ctx.translate(transform.translate.x, transform.translate.y);
  ctx.scale(transform.scale.x, transform.scale.y);
  ctx.rotate(transform.rotate * (transform.anticlockwise ? 1 : -1));
}

// e.g. '#ff0000' → { r: 255, g: 0, b: 0 }
export const hexToRGB = (hex: string): { r: number; g: number; b: number } => {
  const result = /^#([A-Fa-f0-9]{2})([A-Fa-f0-9]{2})([A-Fa-f0-9]{2})/.exec(hex);

  if (result === null) {
    throw Error(`Color ${hex} is not a hex color`);
  } else {
    return {
      r: parseInt(result[1], 16),
      g: parseInt(result[2], 16),
      b: parseInt(result[3], 16),
    };
  }
};

export function applyAppearance(ctx: CanvasRenderingContext2D, appearance: Appearance): void {
  if (appearance instanceof BasicAppearance) {
    if (appearance.fill) {
      const fill = hexToRGB(appearance.fill.color);
      const fA = appearance.fill.alpha;
      ctx.fillStyle = `rgba(${fill.r}, ${fill.g}, ${fill.b}, ${fA})`;
    }

    if (appearance.stroke) {
      const stroke = hexToRGB(appearance.stroke.color);
      const sA = appearance.stroke.alpha;
      ctx.lineWidth = appearance.stroke.width;
      ctx.strokeStyle = `rgba(${stroke.r}, ${stroke.g}, ${stroke.b}, ${sA})`;
    }
  } else if (appearance instanceof LineAppearance) {
    ctx.lineCap = appearance.stroke.cap;
    ctx.lineDashOffset = appearance.stroke.dashOffset;
    ctx.lineJoin = appearance.stroke.join;

    const stroke = hexToRGB(appearance.stroke.color);
    const sA = appearance.stroke.alpha;
    ctx.lineWidth = appearance.stroke.width;
    ctx.strokeStyle = `rgba(${stroke.r}, ${stroke.g}, ${stroke.b}, ${sA})`;
  } else {
    throw new Error(`Unsupported appearance type: ${appearance}`);
  }
}
