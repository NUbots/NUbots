import { Property } from "csstype";
import { computed, observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";
import { Vector3 } from "../../../shared/math/vector3";
import { orthographicCamera, scene, stage } from "../three/builders";
import { Canvas } from "../three/three";

import { ImageViewModel } from "./image_view/view_model";
import { CameraModel } from "./model";
import { CompassViewModel } from "./objects/compass";
import { DistanceViewModel } from "./objects/distance";
import { HorizonViewModel } from "./objects/horizon";
import { Renderable } from "./view";

/**
 * Model containing data for rendering the scene, providing the scene to the THREE component.
 *
 * This class is designed to be extended by more-specific camera view models (like VisionCameraViewModel)
 * to render additional objects beyond the base objects drawn here.
 */
export class CameraViewModel {
  static readonly minZoom = 1;
  static readonly maxZoom = 10;

  @observable accessor canvas: Canvas = { width: 0, height: 0 };
  @observable accessor zoom = 1;
  @observable accessor pan = new Vector2(0, 0);

  @observable accessor lastMousePosition?: Vector2;
  @observable accessor isPanning: boolean = false;

  constructor(protected readonly model: CameraModel) {}

  readonly stage = stage(() => ({ camera: this.camera(), scene: this.scene() }));

  readonly camera = orthographicCamera(() => ({
    left: -1,
    right: 1,
    top: 1,
    bottom: -1,
    near: 0,
    far: 1,
  }));

  readonly scene = scene(() => {
    return {
      children: [
        this.model.drawOptions.drawImage && this.image.image(),
        this.model.drawOptions.drawDistance && this.distance.distance(),
        this.model.drawOptions.drawCompass && this.compass.compass(),
        this.model.drawOptions.drawHorizon && this.horizon.horizon(),
        ...(this.getRenderables?.() ?? []),
      ],
      scale: new Vector3(this.zoom, this.zoom, this.zoom),
      position: this.translation,
    };
  });

  @computed
  get cursor(): Property.Cursor | undefined {
    return this.isPanning ? "grabbing" : this.getCursor?.();
  }

  /**
   * Function for extensions of this class to override
   * to add their own objects to the scene.
   */
  protected getRenderables?(): Renderable[];

  /**
   * Function for extensions of this class to allow them
   * to override the cursor of the camera view.
   */
  protected getCursor?(): Property.Cursor | undefined;

  @computed
  get name(): string {
    return this.model.name;
  }

  @computed
  get drawOptions() {
    return this.model.drawOptions;
  }

  /** Current image offset in GL coords */
  @computed get translation(): Vector3 {
    return new Vector3((-this.pan.x / this.canvas.width) * 2, (this.pan.y / this.canvas.height) * 2, 0);
  }

  @computed
  private get compass(): CompassViewModel {
    // Line width scales with the inverse of the zoom level, so the line looks the same regardless of zoom
    return CompassViewModel.of(this.canvas, this.model.params, this.imageAspectRatio, 5.0);
  }

  @computed
  private get horizon(): HorizonViewModel {
    // Line width scales with the inverse of the zoom level, so the line looks the same regardless of zoom
    return HorizonViewModel.of(this.canvas, this.model.params, this.imageAspectRatio, 10.0);
  }

  @computed
  private get distance(): DistanceViewModel {
    return DistanceViewModel.of(this.canvas, this.model.params, this.imageAspectRatio);
  }

  @computed
  private get image(): ImageViewModel {
    return ImageViewModel.of(this.model.image, this.canvas);
  }

  @computed
  get imageAspectRatio(): number {
    return this.model.image.width / this.model.image.height;
  }
}
