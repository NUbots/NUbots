import { Transform } from '../math/transform'

import { Group } from './object/group'

export type RendererProps = {
  className?: string
  scene: Group
  camera: Transform
  aspectRatio?: number
}
