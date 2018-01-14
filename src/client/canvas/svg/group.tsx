import { observer } from 'mobx-react'
import * as React from 'react'

import { Transform } from '../../math/transform'
import { Group as GroupGeometry } from '../object/group'

import { GeometryView, toSvgTransform } from './svg'

type Props = { model: GroupGeometry, world: Transform }
export const Group = observer(({ model: { children, transform }, world }: Props) => (
  <g transform={toSvgTransform(transform)}>
    {children.map((obj, i) => (<GeometryView key={i} obj={obj} world={world.clone().then(transform.inverse())}/>))}
  </g>
))
