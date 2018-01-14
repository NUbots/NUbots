import * as React from 'react'

import { Transform } from '../../../math/transform'
import { Vector2 } from '../../../math/vector2'
import { BasicAppearance } from '../../appearance/basic_appearance'
import { PolygonGeometry } from '../../geometry/polygon_geometry'
import { Shape } from '../../object/shape'
import { Polygon } from '../polygon'

describe('PolygonSVGRenderer', () => {
  it('renders', () => {
    const model = Shape.of(PolygonGeometry.of([
      Vector2.of(0, 0),
      Vector2.of(0, 1),
      Vector2.of(1, 1),
      Vector2.of(1, 0),
    ]), BasicAppearance.of())
    const world = Transform.of()
    expect(<Polygon model={model} world={world}/>).toMatchSnapshot()
  })
})
