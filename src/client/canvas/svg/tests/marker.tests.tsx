import * as React from 'react'

import { Transform } from '../../../math/transform'
import { BasicAppearance } from '../../appearance/basic_appearance'
import { MarkerGeometry } from '../../geometry/marker_geometry'
import { Shape } from '../../object/shape'
import { Marker } from '../marker'

describe('MarkerSVGRenderer', () => {
  it('renders', () => {
    const model = Shape.of(MarkerGeometry.of(), BasicAppearance.of())
    const world = Transform.of()
    expect(<Marker model={model} world={world}/>).toMatchSnapshot()
  })
})
