import { observable } from "mobx";

import { Image } from "../../../image_decoder/image_decoder";
import { VisualMeshRobotModel } from "../model";

export interface VisualMesh {
  // readonly rows: number[]
  readonly indices: number[];
  readonly neighbours: number[];
  readonly coordinates: number[];
  readonly classifications: { dim: number; values: number[] };
}

type CameraModelOpts = {
  id: number;
  name: string;
};

export class CameraModel {
  readonly id: number;

  @observable.shallow accessor mesh: VisualMesh | undefined;
  @observable.shallow accessor image: Image | undefined;
  @observable accessor name: string;

  constructor(private model: VisualMeshRobotModel, { id, name }: CameraModelOpts) {
    this.id = id;
    this.name = name;
  }

  static of(model: VisualMeshRobotModel, { id, name }: CameraModelOpts) {
    return new CameraModel(model, { id, name });
  }
}
