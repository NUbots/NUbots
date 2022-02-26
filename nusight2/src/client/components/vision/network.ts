import { action, runInAction } from 'mobx'
import { UnreachableError } from '../../../shared/base/unreachable_error'
import { fourcc, fourccToString } from '../../../shared/image_decoder/fourcc'
import { message } from '../../../shared/messages'
import { toSeconds } from '../../../shared/time/timestamp'
import { Matrix4 } from '../../math/matrix4'
import { Vector2 } from '../../math/vector2'
import { Vector3 } from '../../math/vector3'
import { Vector4 } from '../../math/vector4'
import { Network } from '../../network/network'
import { NUsightNetwork } from '../../network/nusight_network'
import { RobotModel } from '../robot/model'
import { CameraModel, CameraParams, GreenHorizon, Lens, Projection } from './camera/model'
import { ImageFormat } from './image'
import { VisionRobotModel } from './model'

export class VisionNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Image, this.onImage)
    this.network.on(message.output.CompressedImage, this.onImage)
    this.network.on(message.vision.VisualMesh, this.onMesh)
    this.network.on(message.vision.Balls, this.onBalls)
    this.network.on(message.vision.Goals, this.onGoals)
    this.network.on(message.vision.GreenHorizon, this.onGreenHorizon)
  }

  static of(nusightNetwork: NUsightNetwork): VisionNetwork {
    const network = Network.of(nusightNetwork)
    return new VisionNetwork(network)
  }

  destroy = () => {
    this.network.off()
  }

  private onImage = async (
    robotModel: RobotModel,
    image: message.input.Image | message.output.CompressedImage,
  ) => {
    const robot = VisionRobotModel.of(robotModel)
    const { id, name, dimensions, format, data, Hcw } = image
    const { projection, focalLength, centre, k } = image?.lens!

    const element = await jpegBufferToImage(data)

    runInAction(() => {
      const camera = robot.cameras.get(id)
      const model = CameraModel.of({
        ...camera,
        id: id,
        name,
        image: {
          type: 'element',
          width: dimensions?.x!,
          height: dimensions?.y!,
          element,
          format: getImageFormat(format),
        },
        params: new CameraParams({
          lens: new Lens({
            projection: getProjection(projection!),
            focalLength: focalLength!,
            centre: Vector2.from(centre),
            distortionCoeffecients: Vector2.from(k),
          }),
          Hcw: Matrix4.from(Hcw),
        }),
      })
      robot.cameras.set(id, (model && camera?.copy(model)) || model)
    })
  }

  @action
  onMesh(robotModel: RobotModel, packet: message.vision.VisualMesh) {
    const robot = VisionRobotModel.of(robotModel)
    const { id, neighbourhood, rays, classifications } = packet
    const camera = robot.cameras.get(id)
    if (!camera) {
      return
    }
    camera.visualmesh = {
      neighbours: neighbourhood?.v!,
      rays: rays?.v!,
      classifications: { dim: classifications?.rows!, values: classifications?.v! },
    }
  }

  @action
  private onBalls(robotModel: RobotModel, packet: message.vision.Balls) {
    const robot = VisionRobotModel.of(robotModel)
    const { id, timestamp, Hcw, balls } = packet
    const camera = robot.cameras.get(id)
    if (!camera) {
      return
    }
    camera.balls = balls.map(ball => ({
      timestamp: toSeconds(timestamp),
      Hcw: Matrix4.from(Hcw),
      cone: {
        axis: Vector3.from(ball.cone?.axis),
        radius: ball.cone?.radius!,
      },
      distance: Math.abs(ball.measurements?.[0].srBCc?.x!),
      colour: Vector4.from(ball.colour),
    }))
  }

  @action
  private onGoals(robotModel: RobotModel, packet: message.vision.Goals) {
    const robot = VisionRobotModel.of(robotModel)
    const { id, timestamp, Hcw, goals } = packet
    const camera = robot.cameras.get(id)
    if (!camera) {
      return
    }
    camera.goals = goals.map(goal => ({
      timestamp: toSeconds(timestamp),
      Hcw: Matrix4.from(Hcw),
      side:
        goal.side === message.vision.Goal.Side.LEFT
          ? 'left'
          : goal.side === message.vision.Goal.Side.RIGHT
          ? 'right'
          : 'unknown',
      post: {
        top: Vector3.from(goal.post?.top),
        bottom: Vector3.from(goal.post?.bottom),
        distance: goal.post?.distance!,
      },
    }))
  }

  @action
  private onGreenHorizon(robotModel: RobotModel, packet: message.vision.GreenHorizon) {
    const robot = VisionRobotModel.of(robotModel)
    const { horizon, Hcw, id } = packet
    const camera = robot.cameras.get(id)
    if (!camera) {
      return
    }
    const greenhorizon = new GreenHorizon({
      horizon: horizon?.map(v => Vector3.from(v)),
      Hcw: Matrix4.from(Hcw),
    })
    camera.greenhorizon = camera.greenhorizon?.copy(greenhorizon) || greenhorizon
  }
}

async function jpegBufferToImage(buffer: ArrayBuffer): Promise<HTMLImageElement> {
  const blob = new Blob([buffer], { type: 'image/jpeg' })
  const url = window.URL.createObjectURL(blob)
  const image = await loadImage(url)
  window.URL.revokeObjectURL(url)
  return image
}

async function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const image = new Image()
    image.onload = () => resolve(image)
    image.onerror = () => reject()
    image.src = url
  })
}

function getImageFormat(format: number): ImageFormat {
  switch (format) {
    case fourcc('JPEG'):
      return ImageFormat.JPEG
    case fourcc('BGGR'):
      return ImageFormat.BGGR
    case fourcc('RGGB'):
      return ImageFormat.RGGB
    case fourcc('GRBG'):
      return ImageFormat.GRBG
    case fourcc('GBRG'):
      return ImageFormat.GBRG
    case fourcc('JPBG'):
      return ImageFormat.JPBG
    case fourcc('JPRG'):
      return ImageFormat.JPRG
    case fourcc('JPGR'):
      return ImageFormat.JPGR
    case fourcc('JPGB'):
      return ImageFormat.JPGB
    case fourcc('PJRG'):
      return ImageFormat.PJRG
    case fourcc('PJGR'):
      return ImageFormat.PJGR
    case fourcc('RGB8'):
      return ImageFormat.RGB8
    case fourcc('PJGB'):
      return ImageFormat.PJGB
    case fourcc('GREY'):
      return ImageFormat.GREY
    case fourcc('GRAY'):
      return ImageFormat.GRAY
    case fourcc('Y8  '):
      return ImageFormat.Y8__
    default:
      throw new Error(`Unsupported format: ${fourccToString(format)}`)
  }
}

function getProjection(projection: message.input.Image.Lens.Projection): Projection {
  switch (projection) {
    case message.input.Image.Lens.Projection.UNKNOWN:
      return Projection.UNKNOWN
    case message.input.Image.Lens.Projection.RECTILINEAR:
      return Projection.RECTILINEAR
    case message.input.Image.Lens.Projection.EQUIDISTANT:
      return Projection.EQUIDISTANT
    case message.input.Image.Lens.Projection.EQUISOLID:
      return Projection.EQUISOLID
    default:
      throw new UnreachableError(projection)
  }
}
