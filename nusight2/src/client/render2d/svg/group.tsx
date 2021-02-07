import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { Group as GroupGeometry } from '../object/group'

import { GeometryView, toSvgTransform } from './rendering'

type Props = { model: GroupGeometry; world: Transform }

export const Group = observer(({ model: { children, transform }, world }: Props) => {
  const objWorld = transform.then(world)
  const elems = children.map((obj, i) => <GeometryView key={i} obj={obj} world={objWorld} />)
  return transform.isIdentity() ? (
    <>{elems}</> // If we have the identity transform forgo the group to save on dom elements
  ) : (
    <g transform={toSvgTransform(transform)}>{elems}</g>
  )
})
