import * as React from 'react'

import { Transform } from '../../../math/transform'
import { BasicAppearance } from '../../appearance/basic_appearance'
import { CircleGeometry } from '../../geometry/circle_geometry'
import { Shape } from '../../object/shape'
import { Circle } from '../circle'

describe('CircleSVGRenderer', () => {
  it('renders', () => {
    const model = Shape.of(CircleGeometry.of(), BasicAppearance.of())
    const world = Transform.of()
    expect(<Circle model={model} world={world}/>).toMatchSnapshot()
  })
})
