import * as React from 'react'

import { Transform } from '../../../math/transform'
import { BasicAppearance } from '../../appearance/basic_appearance'
import { LineGeometry } from '../../geometry/line_geometry'
import { Shape } from '../../object/shape'
import { Line } from '../line'

describe('LineSVGRenderer', () => {
  it('renders', () => {
    const model = Shape.of(LineGeometry.of(), BasicAppearance.of())
    const world = Transform.of()
    expect(<Line model={model} world={world}/>).toMatchSnapshot()
  })
})
