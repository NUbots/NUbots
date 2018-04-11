import * as React from 'react'

import { Transform } from '../../../math/transform'
import { BasicAppearance } from '../../appearance/basic_appearance'
import { PathGeometry } from '../../geometry/path_geometry'
import { Shape } from '../../object/shape'
import { Path } from '../path'

describe('PathSVGRenderer', () => {
  it('renders', () => {
    const model = Shape.of(PathGeometry.of([]), BasicAppearance.of())
    const world = Transform.of()
    expect(<Path model={model} world={world}/>).toMatchSnapshot()
  })
})
