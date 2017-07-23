import { ArrowGeometry } from './arrow_geometry'
import { CircleGeometry } from './circle_geometry'
import { LineGeometry } from './line_geometry'
import { PolygonGeometry } from './polygon_geometry'
import { TextGeometry } from './text_geometry'

export type Geometry =
  ArrowGeometry
  | CircleGeometry
  | LineGeometry
  | PolygonGeometry
  | TextGeometry
