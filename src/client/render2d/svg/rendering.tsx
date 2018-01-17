import { observer } from 'mobx-react'
import * as React from 'react'

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
import { Group as GroupGeometry } from '../object/group'
import { Object2d } from '../object/object2d'
import { Shape } from '../object/shape'

import { Arc } from './arc'
import { Arrow } from './arrow'
import { Circle } from './circle'
import { Group } from './group'
import { Line } from './line'
import { Marker } from './marker'
import { Polygon } from './polygon'
import { Text } from './text'

export function toSvgProps(appearance: Appearance) {
  if (appearance instanceof BasicAppearance) {
    return {
      fill: appearance.fillStyle,
      strokeWidth: appearance.lineWidth,
      stroke: appearance.strokeStyle,
    }
  } else if (appearance instanceof LineAppearance) {
    return {
      strokeLinecap: appearance.lineCap,
      strokeLinejoin: appearance.lineJoin,
      strokeDashoffset: appearance.lineDashOffset,
      strokeWidth: appearance.lineWidth,
      stroke: appearance.strokeStyle,
    }
  } else {
    throw new Error(`Unsupported appearance type ${appearance}`)
  }
}

export function toSvgTransform(transform: Transform): string {
  const s = transform.scale
  const r = (180.0 / Math.PI) * transform.rotate // SVG rotations are in degrees
  const t = transform.translate
  return `translate(${t.x}, ${t.y}) rotate(${r}) scale(${s.x}, ${s.y})`
}

type Props = { obj: Object2d, world: Transform }

export const GeometryView = observer(({ obj, world }: Props): JSX.Element => {
  if (obj instanceof GroupGeometry) {
    return <Group model={obj} world={world}/>
  } else if (obj instanceof Shape) {
    if (obj.geometry instanceof ArcGeometry) {
      return <Arc model={obj} world={world}/>
    } else if (obj.geometry instanceof ArrowGeometry) {
      return <Arrow model={obj} world={world}/>
    } else if (obj.geometry instanceof CircleGeometry) {
      return <Circle model={obj} world={world}/>
    } else if (obj.geometry instanceof LineGeometry) {
      return <Line model={obj} world={world}/>
    } else if (obj.geometry instanceof MarkerGeometry) {
      return <Marker model={obj} world={world}/>
    } else if (obj.geometry instanceof PolygonGeometry) {
      return <Polygon model={obj} world={world}/>
    } else if (obj.geometry instanceof TextGeometry) {
      return <Text model={obj} world={world}/>
    } else {
      throw new Error(`Unsupported geometry type: ${obj}`)
    }
  } else {
    throw new Error(`Unsupported geometry type: ${obj}`)
  }
})
