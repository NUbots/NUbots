import * as fs from 'fs'
import { autorun } from 'mobx'
import * as path from 'path'
import { Vector3 } from 'three'
import { Matrix4 } from 'three'

import { fourcc } from '../../client/image_decoder/fourcc'
import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { Imat4 } from '../../shared/proto/messages'
import { message } from '../../shared/proto/messages'
import { toTimestamp } from '../../shared/time/timestamp'
import { Simulator } from '../simulator'
import { Message } from '../simulator'

import { periodic } from './periodic'
import CompressedImage = message.output.CompressedImage
import Projection = message.output.CompressedImage.Lens.Projection
import Balls = message.vision.Balls
import Side = message.vision.Goal.Side
import Team = message.vision.Goal.Team
import Goals = message.vision.Goals

export class VisionSimulator extends Simulator {
  constructor(nuclearnetClient: NUClearNetClient, private readonly images: Uint8Array[]) {
    super(nuclearnetClient)
  }

  start() {
    const stops = [
      autorun(() => this.send(this.image)),
      autorun(() => this.send(this.goals)),
      autorun(() => this.send(this.balls)),
    ]
    return () => stops.forEach(stop => stop())
  }

  static of({ nuclearnetClient }: { nuclearnetClient: NUClearNetClient }): VisionSimulator {
    const images = Array.from(
      { length: 11 },
      (_, i) => toUint8Array(fs.readFileSync(path.join(__dirname, `images/${i}.jpg`))),
    )
    return new VisionSimulator(nuclearnetClient, images)
  }

  private get image(): Message {
    const time = periodic(10)
    const t = time / 10
    const numImages = this.images.length
    const imageIndex = Math.floor((Math.cos(2 * Math.PI * t) + 1) / 2 * numImages) % numImages
    const data = this.images[imageIndex]
    const Hcw = new Matrix4().makeRotationZ(2 * Math.PI * t)
    return {
      messageType: 'message.output.CompressedImage',
      buffer: CompressedImage.encode({
        format: fourcc('JPEG'),
        dimensions: { x: 712, y: 463 },
        data,
        cameraId: 1,
        name: 'Virtual Camera',
        timestamp: toTimestamp(time),
        Hcw: toProtoMat44(Hcw),
        lens: {
          projection: Projection.RECTILINEAR,
          focalLength: 1,
          fov: { x: 1, y: 1 },
        },
      }).finish(),
    }
  }

  private get balls(): Message {
    const time = periodic(10)
    const t = time / 10
    const Hcw = new Matrix4().makeRotationZ(2 * Math.PI * t)
    const axis = new Vector3(10, 1, 0).normalize().applyMatrix4(new Matrix4().makeRotationX(2 * Math.PI * t))
    return {
      messageType: 'message.vision.Balls',
      buffer: Balls.encode({
        cameraId: 1,
        timestamp: toTimestamp(time),
        Hcw: toProtoMat44(Hcw),
        balls: [{
          cone: {
            axis,
            gradient: Math.cos(Math.PI / 16 * (Math.cos(2 * Math.PI * t) / 5 + 1)),
          },
          measurements: [],
        }],
      }).finish(),
    }
  }

  private get goals(): Message {
    const time = periodic(10)
    const t = time / 10
    const Hcw = new Matrix4().makeRotationZ(2 * Math.PI * t)
    return {
      messageType: 'message.vision.Goals',
      buffer: Goals.encode({
        cameraId: 1,
        timestamp: toTimestamp(time),
        Hcw: toProtoMat44(Hcw),
        goals: [{
          side: Side.UNKNOWN_SIDE,
          team: Team.UNKNOWN_TEAM,
          frustum: {
            tl: new Vector3(10, 1, 2).normalize(),
            tr: new Vector3(10, -1, 2).normalize(),
            bl: new Vector3(10, 1, -2).normalize(),
            br: new Vector3(10, -1, -2).normalize(),
          },
          measurements: [],
        }],
      }).finish(),
    }
  }
}

function toUint8Array(b: Buffer): Uint8Array {
  return new Uint8Array(b.buffer, b.byteOffset, b.byteLength / Uint8Array.BYTES_PER_ELEMENT)
}

function toProtoMat44(m: Matrix4): Imat4 {
  return {
    x: { x: m.elements[0], y: m.elements[1], z: m.elements[2], t: m.elements[3] },
    y: { x: m.elements[4], y: m.elements[5], z: m.elements[6], t: m.elements[7] },
    z: { x: m.elements[8], y: m.elements[9], z: m.elements[10], t: m.elements[11] },
    t: { x: m.elements[12], y: m.elements[13], z: m.elements[14], t: m.elements[15] },
  }
}
