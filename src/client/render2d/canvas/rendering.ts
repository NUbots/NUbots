import { Transform } from '../../math/transform'
import { Appearance } from '../appearance/appearance'
import { BasicAppearance } from '../appearance/basic_appearance'
import { LineAppearance } from '../appearance/line_appearance'
import { ArcGeometry } from '../geometry/arc_geometry'
import { ArrowGeometry } from '../geometry/arrow_geometry'
import { CircleGeometry } from '../geometry/circle_geometry'
import { LineGeometry } from '../geometry/line_geometry'
import { MarkerGeometry } from '../geometry/marker_geometry'
import { PolygonGeometry } from '../geometry/polygon_geometry'
import { TextGeometry } from '../geometry/text_geometry'
import { Group } from '../object/group'
import { Object2d } from '../object/object2d'
import { Shape } from '../object/shape'

import { renderArc } from './arc'
import { renderArrow } from './arrow'
import { renderCircle } from './circle'
import { renderLine } from './line'
import { renderMarker } from './marker'
import { renderPolygon } from './polygon'
import { renderText } from './text'

export function renderObject2d(ctx: CanvasRenderingContext2D, obj: Object2d, world: Transform) {

  if (obj instanceof Group) {
    const objWorld = world.clone().then(obj.transform)

    for (const o of obj.children) {
      ctx.save()
      applyTransform(ctx, obj.transform)
      renderObject2d(ctx, o, objWorld)
      ctx.restore()
    }
  } else if (obj instanceof Shape) {
    if (obj.geometry instanceof ArcGeometry) {
      renderArc(ctx, obj)
    } else if (obj.geometry instanceof ArrowGeometry) {
      renderArrow(ctx, obj)
    } else if (obj.geometry instanceof CircleGeometry) {
      renderCircle(ctx, obj)
    } else if (obj.geometry instanceof LineGeometry) {
      renderLine(ctx, obj)
    } else if (obj.geometry instanceof MarkerGeometry) {
      renderMarker(ctx, obj)
    } else if (obj.geometry instanceof PolygonGeometry) {
      renderPolygon(ctx, obj)
    } else if (obj.geometry instanceof TextGeometry) {
      renderText(ctx, obj, world)
    } else {
      throw new Error(`Unsupported geometry type: ${obj.geometry}`)
    }
  } else {
    throw new Error(`Unsupported Object2d type: ${obj}`)
  }
}

function applyTransform(ctx: CanvasRenderingContext2D, transform: Transform): void {
  ctx.translate(transform.translate.x, transform.translate.y)
  ctx.scale(transform.scale.x, transform.scale.y)
  ctx.rotate(transform.rotate * (transform.anticlockwise ? 1 : -1))
}

export function applyAppearance(ctx: CanvasRenderingContext2D, appearance: Appearance): void {

  if (appearance instanceof BasicAppearance) {
    ctx.fillStyle = appearance.fillStyle
    ctx.lineWidth = appearance.lineWidth
    ctx.strokeStyle = appearance.strokeStyle
  } else if (appearance instanceof LineAppearance) {
    ctx.lineCap = appearance.lineCap
    ctx.lineDashOffset = appearance.lineDashOffset
    ctx.lineJoin = appearance.lineJoin
    ctx.lineWidth = appearance.lineWidth
    ctx.strokeStyle = appearance.strokeStyle
  } else {
    throw new Error(`Unsupported appearance type: ${appearance}`)
  }
}
