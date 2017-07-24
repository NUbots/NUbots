import { Transform } from '../../math/transform'

export type Object2d = {
  children: Object2d[]
  transform: Transform
  add(obj: Object2d): void
}
