import { action } from 'mobx'

import { FieldModel } from './model'

export class FieldController {
  public static of(): FieldController {
    return new FieldController()
  }

  @action
  public onFieldResize(model: FieldModel, width: number, height: number) {
    const fieldWidth = model.fieldWidth
    const fieldLength = model.fieldLength
    const scaleX = fieldLength / width
    const scaleY = fieldWidth / height

    const canvasAspect = width / height
    const fieldAspect = fieldLength / fieldWidth
    // Ensures the canvas is scaled such that the entire field is always visible, regardless of aspect ratio.
    const scale = canvasAspect > fieldAspect ? scaleY : scaleX

    model.camera.scale.x = scale
    model.camera.scale.y = -scale

    // Translate by half of the canvas width and height so that the field appears in the center.
    model.camera.translate.x = -width * 0.5
    model.camera.translate.y = -height * 0.5
  }
}
