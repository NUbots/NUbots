import { action } from 'mobx'
import { computed } from 'mobx'
import { observable } from 'mobx'

import { Transform } from '../../math/transform'
import { Appearance } from '../appearance/appearance'
import { BasicAppearance } from '../appearance/basic_appearance'
import { ArcGeometry } from '../geometry/arc_geometry'
import { ArrowGeometry } from '../geometry/arrow_geometry'
import { CircleGeometry } from '../geometry/circle_geometry'
import { LineGeometry } from '../geometry/line_geometry'
import { PolygonGeometry } from '../geometry/polygon_geometry'
import { TextGeometry } from '../geometry/text_geometry'

import { Group } from './group'
import { GroupOpts } from './group'
import { Object2d } from './object2d'

export type Geometry =
  ArcGeometry
  | ArrowGeometry
  | CircleGeometry
  | LineGeometry
  | PolygonGeometry
  | TextGeometry

export type ShapeOpts = {
  appearance: Appearance
  geometry: Geometry
} & GroupOpts

export class Shape implements Object2d {
  @observable public appearance: Appearance
  @observable public geometry: Geometry
  @observable private group: Group

  constructor(opts: ShapeOpts) {
    this.appearance = opts.appearance
    this.geometry = opts.geometry
    this.group = Group.of(opts)
  }

  public static of(geometry: Geometry, appearance: Appearance = BasicAppearance.of()) {
    return new Shape({
      appearance,
      children: [],
      geometry,
      transform: Transform.of(),
    })
  }

  public add(obj: Object2d): void {
    this.group.add(obj)
  }

  @computed
  public get children(): Object2d[] {
    return this.group.children
  }

  @computed
  public get transform(): Transform {
    return this.group.transform
  }
}
