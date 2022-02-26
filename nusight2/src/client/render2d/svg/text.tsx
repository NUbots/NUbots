import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { TextGeometry } from '../geometry/text_geometry'
import { Shape } from '../object/shape'

import { toSvgProps, toSvgTransform } from './rendering'

type Props = { model: Shape<TextGeometry>; world: Transform }
export const Text = observer(({ model: { geometry, appearance }, world }: Props) => {
  const { x, y, fontFamily, fontSize, text, textAlign, textBaseline, worldAlignment, worldScale } =
    geometry

  const t = Transform.of({
    translate: { x, y },
    scale: worldScale
      ? { x: 1 / world.scale.x, y: 1 / world.scale.y }
      : { x: Math.sign(world.scale.x), y: Math.sign(world.scale.y) },
    rotate: worldAlignment ? -world.rotate : 0,
  })

  // TODO (Houliston): Calculate fontSize such that the text fits within the given maxWidth
  // currently the font set here is correct for the dashboard view, but the canvas view is able to dynamically scale it
  return (
    <g transform={toSvgTransform(t)}>
      <text
        fontFamily={fontFamily}
        fontSize={fontSize}
        textAnchor={textAlign}
        dominantBaseline={textBaseline}
        {...toSvgProps(appearance)}
      >
        {text}
      </text>
    </g>
  )
})
