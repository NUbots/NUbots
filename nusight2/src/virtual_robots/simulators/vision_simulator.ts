import { CompressedImage, CompressedImage_Lens_ProjectionEnum } from "@proto/message/output/CompressedImage";
import { Ball_MeasurementTypeEnum, Balls } from "@proto/message/vision/Ball";
import { Goal_SideEnum, Goals } from "@proto/message/vision/Goal";
import fs from "fs";
import { autorun } from "mobx";
import { computedFn } from "mobx-utils";
import { Matrix4, Vector3 } from "three";

import { FieldDimensions } from "../../shared/field/dimensions";
import { fourcc } from "../../shared/image_decoder/fourcc";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { Simulator } from "../simulator";
import { Message } from "../simulator";

import image0 from "./images/0.jpg";
import image1 from "./images/1.jpg";
import image2 from "./images/2.jpg";
import image3 from "./images/3.jpg";
import image4 from "./images/4.jpg";
import image5 from "./images/5.jpg";
import image6 from "./images/6.jpg";
import image7 from "./images/7.jpg";
import image8 from "./images/8.jpg";
import image9 from "./images/9.jpg";
import image10 from "./images/10.jpg";
import { periodic } from "./periodic";

export class VisionSimulator extends Simulator {
  constructor(
    nuclearnetClient: NUClearNetClient,
    private readonly images: Uint8Array[],
  ) {
    super(nuclearnetClient);
  }

  start() {
    const stops = [
      autorun(() => {
        this.send(this.image(1));
        this.send(this.image(2));
      }),
      autorun(() => this.send(this.goals)),
      autorun(() => this.send(this.balls)),
    ];
    return () => stops.forEach((stop) => stop());
  }

  static of({ nuclearnetClient }: { nuclearnetClient: NUClearNetClient }): VisionSimulator {
    const urls = [image0, image1, image2, image3, image4, image5, image6, image7, image8, image9, image10];
    const images = urls.map((url) => toUint8Array(fs.readFileSync(url)));
    return new VisionSimulator(nuclearnetClient, images);
  }

  private image = computedFn((id: number): Message => {
    const time = periodic(10);
    const t = time / 10;
    const numImages = this.images.length;
    const imageIndex = Math.floor(((Math.cos(2 * Math.PI * t) + 1) / 2) * numImages) % numImages;
    const data = this.images[imageIndex];
    const Hcw = new Matrix4().makeRotationZ(2 * Math.PI * t);
    return {
      messageType: "message.output.CompressedImage",
      buffer: new CompressedImage({
        format: fourcc("JPEG"),
        dimensions: { x: 712, y: 463 },
        data,
        id,
        name: `Virtual Camera #${id}`,
        timestamp: toProtoTimestamp(time),
        Hcw: toProtoMat44(Hcw),
        lens: {
          projection: CompressedImage_Lens_ProjectionEnum.RECTILINEAR,
          focalLength: 415 / 712,
          fov: 1,
        },
      }).toBinary(),
    };
  });

  private get balls(): Message {
    const time = periodic(10);
    const t = time / 10;
    const Hcw = new Matrix4().makeRotationZ(2 * Math.PI * t);
    const uBCc = new Vector3(10, 1, 0).normalize().applyMatrix4(new Matrix4().makeRotationX(2 * Math.PI * t));
    return {
      messageType: "message.vision.Balls",
      buffer: new Balls({
        id: 1,
        timestamp: toProtoTimestamp(time),
        Hcw: toProtoMat44(Hcw),
        balls: [
          {
            uBCc,
            radius: Math.cos((Math.PI / 16) * (Math.cos(2 * Math.PI * t) / 5 + 1)),
            measurements: [
              {
                type: Ball_MeasurementTypeEnum.ANGULAR,
                rBCc: new Vector3(1, 0, 0),
              },
            ],
          },
        ],
      }).toBinary(),
    };
  }

  private get goals(): Message {
    const time = periodic(10);
    const t = time / 10;
    const Hcw = new Matrix4().makeRotationZ(2 * Math.PI * t);
    const { goalCrossbarHeight } = FieldDimensions.of();
    return {
      messageType: "message.vision.Goals",
      buffer: new Goals({
        id: 1,
        timestamp: toProtoTimestamp(time),
        Hcw: toProtoMat44(Hcw),
        goals: [
          {
            side: Goal_SideEnum.RIGHT,
            post: {
              top: new Vector3(5, -1, goalCrossbarHeight / 2).normalize(),
              bottom: new Vector3(5, -1, -goalCrossbarHeight / 2).normalize(),
              distance: new Vector3(5, -1, -goalCrossbarHeight / 2).length(),
            },
            measurements: [],
          },
          {
            side: Goal_SideEnum.LEFT,
            post: {
              top: new Vector3(5, 1, goalCrossbarHeight / 2).normalize(),
              bottom: new Vector3(5, 1, -goalCrossbarHeight / 2).normalize(),
              distance: new Vector3(5, 1, -goalCrossbarHeight / 2).length(),
            },
            measurements: [],
          },
          {
            side: Goal_SideEnum.UNKNOWN_SIDE,
            post: {
              top: new Vector3(5, 0, goalCrossbarHeight / 2).normalize(),
              bottom: new Vector3(5, 0, -goalCrossbarHeight / 2).normalize(),
              distance: new Vector3(5, 0, -goalCrossbarHeight / 2).length(),
            },
            measurements: [],
          },
        ],
      }).toBinary(),
    };
  }
}

function toUint8Array(b: Buffer): Uint8Array {
  return new Uint8Array(b.buffer, b.byteOffset, b.byteLength / Uint8Array.BYTES_PER_ELEMENT);
}

function toProtoMat44(m: Matrix4) {
  return {
    x: { x: m.elements[0], y: m.elements[1], z: m.elements[2], t: m.elements[3] },
    y: { x: m.elements[4], y: m.elements[5], z: m.elements[6], t: m.elements[7] },
    z: { x: m.elements[8], y: m.elements[9], z: m.elements[10], t: m.elements[11] },
    t: { x: m.elements[12], y: m.elements[13], z: m.elements[14], t: m.elements[15] },
  };
}

function toProtoTimestamp(seconds: number) {
  return { seconds: BigInt(Math.floor(seconds)), nanos: Math.floor((seconds * 1e9) % 1e9) };
}
