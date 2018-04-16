import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'
import { Container } from 'pixi.js'
import { DisplayObject } from 'pixi.js'

import { Transform } from '../../math/transform'
import { Appearance } from '../appearance/appearance'
import { BasicAppearance } from '../appearance/basic_appearance'
import { LineAppearance } from '../appearance/line_appearance'
import { ArcGeometry } from '../geometry/arc_geometry'
import { ArrowGeometry } from '../geometry/arrow_geometry'
import { CircleGeometry } from '../geometry/circle_geometry'
import { LineGeometry } from '../geometry/line_geometry'
import { MarkerGeometry } from '../geometry/marker_geometry'
import { PathGeometry } from '../geometry/path_geometry'
import { PolygonGeometry } from '../geometry/polygon_geometry'
import { TextGeometry } from '../geometry/text_geometry'
import { Geometry } from '../object/geometry'
import { Group } from '../object/group'
import { Shape } from '../object/shape'

import { renderArc } from './arc'
import { renderArrow } from './arrow'
import { renderCircle } from './circle'
import { renderLine } from './line'
import { renderMarker } from './marker'
import { renderPath } from './path'
import { renderPolygon } from './polygon'
import { renderText } from './text'

export const pixiObject = createTransformer((obj: Group | Shape<Geometry>): DisplayObject => {

  if (obj instanceof Group) {
    const g = new Container()

    const { transform: { scale, translate, rotate } } = obj
    g.scale.x = scale.x
    g.scale.y = scale.x
    g.x = translate.x
    g.y = translate.y
    g.rotation = rotate

    obj.children.forEach(o => {
      g.addChild(pixiObject(o))
    })

    return g

  } else if (obj instanceof Shape) {
    if (obj.geometry instanceof ArcGeometry) {
      return renderArc(obj as Shape<ArcGeometry>)
    } else if (obj.geometry instanceof ArrowGeometry) {
      return renderArrow(obj as Shape<ArrowGeometry>)
    } else if (obj.geometry instanceof CircleGeometry) {
      return renderCircle(obj as Shape<CircleGeometry>)
    } else if (obj.geometry instanceof LineGeometry) {
      return renderLine(obj as Shape<LineGeometry>)
    } else if (obj.geometry instanceof MarkerGeometry) {
      return renderMarker(obj as Shape<MarkerGeometry>)
    } else if (obj.geometry instanceof PathGeometry) {
      return renderPath(obj as Shape<PathGeometry>)
    } else if (obj.geometry instanceof PolygonGeometry) {
      return renderPolygon(obj as Shape<PolygonGeometry>)
    } else if (obj.geometry instanceof TextGeometry) {
      return renderText(obj as Shape<TextGeometry>)
    } else {
      throw new Error(`Unsupported geometry type: ${obj.geometry}`)
    }
  } else {
    throw new Error(`Unsupported Object2d type: ${obj}`)
  }
})

export function applyAppearance(obj: Graphics, appearance: Appearance, draw: (obj: Graphics) => void): void {

  if (appearance instanceof BasicAppearance) {
    if (appearance.stroke) {
      obj.lineStyle(appearance.stroke.width, parseInt(appearance.stroke.color.slice(1), 16), appearance.stroke.alpha)
    } else {
      obj.lineStyle(0, 0, 0)
    }

    if (appearance.fill) {
      obj.beginFill(parseInt(appearance.fill.color.slice(1), 16), appearance.fill.alpha)
    }

    draw(obj)

    if (appearance.fill) {
      obj.endFill()
    }

  } else if (appearance instanceof LineAppearance) {

    obj.lineStyle(appearance.stroke.width, parseInt(appearance.stroke.color.slice(1), 16), appearance.stroke.alpha)
    draw(obj)

    // TODO These properties don't have easy analogs in pixi
    // appearance.stroke.cap
    // appearance.stroke.dashOffset
    // appearance.stroke.join
    // appearance.stroke.width
  } else {
    throw new Error(`Unsupported appearance type: ${appearance}`)
  }
}
