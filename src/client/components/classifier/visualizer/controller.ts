import { action } from 'mobx'

import { Vector2 } from '../../../math/vector2'

import { VisualizerModel } from './model'

const PI2 = Math.PI / 2
const DegToRad = Math.PI / 180

export class VisualizerController {
  constructor(private model: VisualizerModel) {}

  static of(model: VisualizerModel) {
    return new VisualizerController(model)
  }

  @action.bound
  onMouseDown(x: number, y: number) {
    this.model.dragger = Dragger.of(this.model, Vector2.of(x, y))
  }

  @action.bound
  onMouseMove(x: number, y: number) {
    const dragger = this.model.dragger
    if (dragger) {
      dragger.to = Vector2.of(x, y)
    }
  }

  @action.bound
  onMouseUp() {
    this.model.dragger = undefined
  }

  @action.bound
  onWheel(deltaY: number, preventDefault: () => void) {
    preventDefault()
    this.model.camera.distance = clamp(this.model.camera.distance + deltaY / 500, 0.01, 10)
  }
}

export class Dragger {
  private readonly from: Vector2
  // tslint:disable-next-line variable-name
  private _to: Vector2
  private readonly fromCamera: { azimuth: number; elevation: number }

  constructor(
    private readonly model: VisualizerModel,
    from: Vector2,
    to: Vector2,
    fromCamera: { azimuth: number; elevation: number },
  ) {
    this.from = from
    this._to = to
    this.fromCamera = fromCamera
  }

  static of(model: VisualizerModel, from: Vector2) {
    return new Dragger(model, from, from, {
      azimuth: model.camera.azimuth,
      elevation: model.camera.elevation,
    })
  }

  set to(to: Vector2) {
    this._to = to
    this.update()
  }

  private update() {
    const diff = this.diff()
    this.model.camera.azimuth = this.fromCamera.azimuth - diff.x
    this.model.camera.elevation = clamp(this.fromCamera.elevation - diff.y, -PI2 + 0.01, PI2 - 0.01)
  }

  private diff(): Vector2 {
    return this._to.subtract(this.from).multiplyScalar(0.5 * DegToRad)
  }
}

function clamp(x: number, min: number = -Infinity, max: number = Infinity) {
  return Math.min(max, Math.max(min, x))
}
