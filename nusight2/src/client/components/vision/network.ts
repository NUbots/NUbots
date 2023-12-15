import { action, runInAction } from "mobx";

import { UnreachableError } from "../../../shared/base/unreachable_error";
import { fourcc, fourccToString } from "../../../shared/image_decoder/fourcc";
import { Matrix4 } from "../../../shared/math/matrix4";
import { Projection } from "../../../shared/math/projection";
import { Vector2 } from "../../../shared/math/vector2";
import { Vector3 } from "../../../shared/math/vector3";
import { Vector4 } from "../../../shared/math/vector4";
import { message } from "../../../shared/messages";
import { toSeconds } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { CameraParams } from "../camera/camera_params";
import { ImageFormat } from "../camera/image";
import { Lens } from "../camera/lens";
import { RobotModel } from "../robot/model";

import { VisionRobotModel } from "./model";
import { GreenHorizonModel } from "./vision_camera/green_horizon";
import { VisionCameraModel } from "./vision_camera/model";

export class VisionNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Image, this.onImage);
    this.network.on(message.output.CompressedImage, this.onImage);
    this.network.on(message.vision.VisualMesh, this.onMesh);
    this.network.on(message.vision.Balls, this.onBalls);
    this.network.on(message.vision.Goals, this.onGoals);
    this.network.on(message.vision.GreenHorizon, this.onGreenHorizon);
  }

  static of(nusightNetwork: NUsightNetwork): VisionNetwork {
    const network = Network.of(nusightNetwork);
    return new VisionNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  private onImage = async (robotModel: RobotModel, image: message.input.Image | message.output.CompressedImage) => {
    // Image data or lens will be null for empty NBS packets (which are emitted by the nbs scrubber when there's no
    // image packet at a requested timestamp). Ignoring for now, but we'd want to draw a blank image in the future.
    if (!image.data || !image.lens) {
      return;
    }

    const robot = VisionRobotModel.of(robotModel);
    const { id, name, dimensions, format, data, Hcw } = image;
    const { projection, focalLength, centre, k } = image.lens!;

    const bitmap = await jpegBufferToBitmap(data);

    runInAction(() => {
      const camera = robot.cameras.get(id);
      const model = VisionCameraModel.of({
        ...camera,
        id: id,
        name,
        image: {
          type: "bitmap",
          bitmap,
          width: dimensions?.x!,
          height: dimensions?.y!,
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
      });
      robot.cameras.set(id, (model && camera?.copy(model)) || model);
    });
  };

  @action
  onMesh(robotModel: RobotModel, packet: message.vision.VisualMesh) {
    const robot = VisionRobotModel.of(robotModel);
    const { id, neighbourhood, uPCw, classifications } = packet;
    const camera = robot.cameras.get(id);
    if (!camera) {
      return;
    }
    camera.visualMesh = {
      neighbours: neighbourhood?.v!,
      rays: uPCw?.v!,
      classifications: { dim: classifications?.rows!, values: classifications?.v! },
    };
  }

  @action
  private onBalls(robotModel: RobotModel, packet: message.vision.Balls) {
    const robot = VisionRobotModel.of(robotModel);
    const { id, timestamp, Hcw, balls } = packet;
    const camera = robot.cameras.get(id);
    if (!camera) {
      return;
    }
    camera.balls = balls.map((ball) => ({
      timestamp: toSeconds(timestamp),
      Hcw: Matrix4.from(Hcw),
      cone: {
        axis: Vector3.from(ball.uBCc),
        radius: ball.radius!,
      },
      distance: Math.hypot(ball.measurements?.[0].rBCc?.x!, ball.measurements?.[0].rBCc?.y!, ball.measurements?.[0].rBCc?.z!),
      colour: Vector4.from(ball.colour),
    }));
  }

  @action
  private onGoals(robotModel: RobotModel, packet: message.vision.Goals) {
    const robot = VisionRobotModel.of(robotModel);
    const { id, timestamp, Hcw, goals } = packet;
    const camera = robot.cameras.get(id);
    if (!camera) {
      return;
    }
    camera.goals = goals.map((goal) => ({
      timestamp: toSeconds(timestamp),
      Hcw: Matrix4.from(Hcw),
      side:
        goal.side === message.vision.Goal.Side.LEFT
          ? "left"
          : goal.side === message.vision.Goal.Side.RIGHT
          ? "right"
          : "unknown",
      post: {
        top: Vector3.from(goal.post?.top),
        bottom: Vector3.from(goal.post?.bottom),
        distance: goal.post?.distance!,
      },
    }));
  }

  @action
  private onGreenHorizon(robotModel: RobotModel, packet: message.vision.GreenHorizon) {
    const robot = VisionRobotModel.of(robotModel);
    const { horizon, Hcw, id } = packet;
    const camera = robot.cameras.get(id);
    if (!camera) {
      return;
    }
    const greenHorizon = new GreenHorizonModel({
      horizon: horizon?.map((v) => Matrix4.from(Hcw).invert().decompose().translation.subtract(Vector3.from(v))),
      Hcw: Matrix4.from(Hcw),
    });
    camera.greenHorizon = camera.greenHorizon?.copy(greenHorizon) || greenHorizon;
  }
}

async function jpegBufferToBitmap(buffer: ArrayBuffer): Promise<ImageBitmap> {
  const blob = new Blob([buffer], { type: "image/jpeg" });
  return createImageBitmap(blob, {
    colorSpaceConversion: "none",
    premultiplyAlpha: "none",
  });
}

function getImageFormat(format: number): ImageFormat {
  switch (format) {
    case fourcc("JPEG"):
      return ImageFormat.JPEG;
    case fourcc("BGGR"):
      return ImageFormat.BGGR;
    case fourcc("RGGB"):
      return ImageFormat.RGGB;
    case fourcc("GRBG"):
      return ImageFormat.GRBG;
    case fourcc("GBRG"):
      return ImageFormat.GBRG;
    case fourcc("JPBG"):
      return ImageFormat.JPBG;
    case fourcc("JPRG"):
      return ImageFormat.JPRG;
    case fourcc("JPGR"):
      return ImageFormat.JPGR;
    case fourcc("JPGB"):
      return ImageFormat.JPGB;
    case fourcc("PJRG"):
      return ImageFormat.PJRG;
    case fourcc("PJGR"):
      return ImageFormat.PJGR;
    case fourcc("RGB8"):
      return ImageFormat.RGB8;
    case fourcc("PJGB"):
      return ImageFormat.PJGB;
    case fourcc("GREY"):
      return ImageFormat.GREY;
    case fourcc("GRAY"):
      return ImageFormat.GRAY;
    case fourcc("Y8  "):
      return ImageFormat.Y8__;
    default:
      throw new Error(`Unsupported format: ${fourccToString(format)}`);
  }
}

function getProjection(projection: message.input.Image.Lens.Projection): Projection {
  switch (projection) {
    case message.input.Image.Lens.Projection.UNKNOWN:
      return Projection.UNKNOWN;
    case message.input.Image.Lens.Projection.RECTILINEAR:
      return Projection.RECTILINEAR;
    case message.input.Image.Lens.Projection.EQUIDISTANT:
      return Projection.EQUIDISTANT;
    case message.input.Image.Lens.Projection.EQUISOLID:
      return Projection.EQUISOLID;
    default:
      throw new UnreachableError(projection);
  }
}
