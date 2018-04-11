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
import { Group } from '../object/group'
import { Object2d } from '../object/object2d'
import { Shape } from '../object/shape'

import { renderArc } from './arc'
import { renderArrow } from './arrow'
import { renderCircle } from './circle'
import { renderLine } from './line'
import { renderMarker } from './marker'
import { renderPath } from './path'
import { renderPolygon } from './polygon'
import { renderText } from './text'

export const pixiObject = createTransformer((obj: Object2d): DisplayObject => {

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
      return renderArc(obj)
    } else if (obj.geometry instanceof ArrowGeometry) {
      return renderArrow(obj)
    } else if (obj.geometry instanceof CircleGeometry) {
      return renderCircle(obj)
    } else if (obj.geometry instanceof LineGeometry) {
      return renderLine(obj)
    } else if (obj.geometry instanceof MarkerGeometry) {
      return renderMarker(obj)
    } else if (obj.geometry instanceof PathGeometry) {
      return renderPath(obj)
    } else if (obj.geometry instanceof PolygonGeometry) {
      return renderPolygon(obj)
    } else if (obj.geometry instanceof TextGeometry) {
      return renderText(obj)
    } else {
      throw new Error(`Unsupported geometry type: ${obj.geometry}`)
    }
  } else {
    throw new Error(`Unsupported Object2d type: ${obj}`)
  }
})

const toPixiColor = (style: string): {
  color: number, // range: [0, 0xFFFFFF]
  alpha: number // range: [0, 1]
} => {
  if (style === 'transparent') {
    return { color: 0, alpha: 0 }
  }
  // Colors of the form #FFFFFF
  const split = /^#([A-Fa-f0-9]{6})$/.exec(style)
  if (split !== null) {
    return { color: parseInt(split[1], 16), alpha: 1 }
  }

  throw new Error('Pixi cannot handle non hex colours')
}

export function applyAppearance(obj: Graphics, appearance: Appearance, draw: (obj: Graphics) => void): void {

  if (appearance instanceof BasicAppearance) {
    const line = toPixiColor(appearance.strokeStyle)
    const fill = toPixiColor(appearance.fillStyle)
    obj.lineStyle(appearance.lineWidth, line.color, line.alpha)
    obj.beginFill(fill.color, fill.alpha)
    draw(obj)
    obj.endFill()
  } else if (appearance instanceof LineAppearance) {
    const line = toPixiColor(appearance.strokeStyle)
    obj.lineStyle(appearance.lineWidth, line.color, line.alpha)
    draw(obj)

    // TODO: Support lineCap, lineCap, lineJoin, lineWidth, strokeStyle
  } else {
    throw new Error(`Unsupported appearance type: ${appearance}`)
  }
}
