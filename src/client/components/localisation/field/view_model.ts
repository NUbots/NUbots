import { createTransformer } from 'mobx'
import { computed } from 'mobx'
import { PlaneGeometry } from 'three'
import { Matrix4 } from 'three'
import { MeshBasicMaterial } from 'three'
import { RingGeometry } from 'three'
import { Object3D } from 'three'
import { Mesh } from 'three'
import { Geometry } from 'three'
import { FieldModel } from './model'

export class FieldViewModel {
  public constructor(private model: FieldModel) {
  }

  public static of = createTransformer((model: FieldModel): FieldViewModel => {
    return new FieldViewModel(model)
  })

  @computed
  public get field() {
    const field = new Object3D()
    field.add(this.ground)
    field.add(this.fieldLines)
    return field
  }

  @computed
  private get ground() {
    return new Mesh(this.groundGeometry, this.groundMaterial)
  }

  @computed
  private get groundGeometry() {
    const geometry = new PlaneGeometry(
      this.model.dimensions.fieldLength + this.model.dimensions.borderStripMinWidth * 2,
      this.model.dimensions.fieldWidth + this.model.dimensions.borderStripMinWidth * 2,
    )
    return geometry
  }

  @computed
  private get groundMaterial() {
    return new MeshBasicMaterial({ color: this.model.fieldColor })
  }

  @computed
  private get fieldLines() {
    const fieldLines = new Mesh(this.fieldLinesGeometry, this.fieldLinesMaterial)
    fieldLines.position.z = 0.001
    return fieldLines
  }

  @computed
  private get fieldLinesGeometry() {
    const geometry = new Geometry()
    const centerCircle = this.centerCircle

    const fieldWidth = this.model.dimensions.fieldWidth
    const fieldLength = this.model.dimensions.fieldLength
    const lineWidth = this.model.dimensions.lineWidth

    const halfLength = fieldLength * 0.5
    const halfWidth = fieldWidth * 0.5

    const blueHalf = this.buildRectangle(-halfLength, -halfWidth, halfLength, fieldWidth, lineWidth)
    const yellowHalf = this.buildRectangle(0, -halfWidth, halfLength, fieldWidth, lineWidth)

    const identity = new Matrix4()
    geometry.merge(centerCircle, identity)
    geometry.merge(blueHalf, identity)
    geometry.merge(yellowHalf, identity)

    return geometry
  }

  @computed
  private get centerCircle() {
    return new RingGeometry(
      (this.model.dimensions.centerCircleDiameter - this.model.dimensions.lineWidth) * 0.5,
      (this.model.dimensions.centerCircleDiameter + this.model.dimensions.lineWidth) * 0.5,
      128,
    )
  }

  @computed
  private get fieldLinesMaterial() {
    return new MeshBasicMaterial({ color: this.model.lineColor })
  }

  private buildRectangle(x: number, y: number, w: number, h: number, lw: number) {
    const x1 = x - lw * 0.5
    const x2 = x + w + lw * 0.5
    const topLine = this.buildHorizontalLine(x1, x2, y, lw)
    const bottomLine = this.buildHorizontalLine(x1, x2, y + h, lw)

    const leftLine = this.buildVerticalLine(y, y + h, x, lw)
    const rightLine = this.buildVerticalLine(y, y + h, x + w, lw)

    const identity = new Matrix4()
    const rect = new Geometry()
    rect.merge(topLine, identity)
    rect.merge(bottomLine, identity)
    rect.merge(leftLine, identity)
    rect.merge(rightLine, identity)
    return rect
  }

  private buildHorizontalLine(x1: number, x2: number, y: number, width: number) {
    const length = x2 - x1
    const hLine = new PlaneGeometry(length, width)
    hLine.applyMatrix(new Matrix4().makeTranslation(x1 + length * 0.5, y, 0))
    return hLine
  }

  private buildVerticalLine(y1: number, y2: number, x: number, width: number) {
    const length = y2 - y1
    const vLine = new PlaneGeometry(width, length)
    vLine.applyMatrix(new Matrix4().makeTranslation(x, y1 + length * 0.5, 0))
    return vLine
  }
}
