import { action } from 'mobx'

import { message } from '../../../shared/proto/messages'
import { Matrix4 } from '../../math/matrix4'
import { Network } from '../../network/network'
import { NUsightNetwork } from '../../network/nusight_network'
import { RobotModel } from '../robot/model'

import { CameraModel } from './camera/model'
import { VisionRobotModel } from './model'
import Image = message.input.Image
import CompressedImage = message.output.CompressedImage

export class VisionNetwork {

  constructor(private network: Network) {
    this.network.on(Image, this.onImage)
    this.network.on(CompressedImage, this.onImage)
  }

  static of(nusightNetwork: NUsightNetwork): VisionNetwork {
    const network = Network.of(nusightNetwork)
    return new VisionNetwork(network)
  }

  destroy() {
    this.network.off()
  }

  @action
  private onImage = (robotModel: RobotModel, image: Image | CompressedImage) => {
    const robot = VisionRobotModel.of(robotModel)
    const { cameraId, name, dimensions, format, data, Hcw } = image

    // TODO DEBUG for now the recording in use has no lens information, so we are attaching it here
    const lens = {
      projection: Image.Lens.Projection.EQUIDISTANT,
      focalLength: (1.0 / 0.0026997136600899543) / 1280.0,
    }
    lens.projection = image!.lens!.projection || lens.projection
    lens.focalLength = image!.lens!.focalLength || lens.focalLength

    let camera = robot.cameras.get(cameraId)
    if (!camera) {
      camera = CameraModel.of(robot, {
        id: cameraId,
        name,
      })
      robot.cameras.set(cameraId, camera)
    }
    camera.image = {
      width: dimensions!.x!,
      height: dimensions!.y!,
      format,
      data,
      lens: {
        projection: lens!.projection!,
        focalLength: lens!.focalLength!,
      },
      Hcw: Matrix4.from(Hcw),
    }
    camera.name = name
  }
}
