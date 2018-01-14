import { observer } from 'mobx-react'
import * as React from 'react'

import { Transform } from '../../math/transform'
import { TextGeometry } from '../geometry/text_geometry'
import { Shape } from '../object/shape'

import { toSvgProps, toSvgTransform } from './svg'

type Props = { model: Shape<TextGeometry>, world: Transform }
export const Text = observer(({ model: { geometry, appearance }, world }: Props) => {
  const { x, y, fontFamily, text, textAlign, textBaseline, alignToView } = geometry

  const t = alignToView ? Transform.of({
    scale: { x: Math.sign(world.scale.x), y: Math.sign(world.scale.y) },
    rotate: -world.rotate,
  }) : Transform.of()

  // TODO (Houliston): Calculate fontSize such that the text fits within the given maxWidth
  // currently the font set here is correct for the dashboard view, but the canvas view is able to dynamically scale it
  return (
    <text
      x={x}
      y={y}
      fontFamily={fontFamily}
      fontSize={'0.015rem'}
      textAnchor={textAlign}
      dominantBaseline={textBaseline}
      transform={toSvgTransform(t)}
      {...toSvgProps(appearance)}
    >
      {text}
    </text>
  )
})
