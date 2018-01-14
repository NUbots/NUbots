import * as React from 'react'

import { Transform } from '../../../math/transform'
import { BasicAppearance } from '../../appearance/basic_appearance'
import { ArrowGeometry } from '../../geometry/arrow_geometry'
import { Shape } from '../../object/shape'
import { Arrow } from '../arrow'

describe('ArrowSVGRenderer', () => {
  it('renders', () => {
    const model = Shape.of(ArrowGeometry.of(), BasicAppearance.of())
    const world = Transform.of()
    expect(<Arrow model={model} world={world}/>).toMatchSnapshot()
  })
})
