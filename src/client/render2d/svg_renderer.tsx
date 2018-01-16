import { observer } from 'mobx-react'
import * as React from 'react'

import { Transform } from '../math/transform'

import { Group as GroupGeometry } from './object/group'
import { Group } from './svg/group'
import { toSvgTransform } from './svg/svg'

export type SVGRendererProps = {
  className: string
  scene: GroupGeometry
  camera: Transform
}

export const SVGRenderer = observer(({ className, scene, camera }: SVGRendererProps) => (
  <svg className={className}>
    <g transform={toSvgTransform(camera.inverse())}>
      <Group model={scene} world={camera.inverse()}/>
    </g>
  </svg>
))
