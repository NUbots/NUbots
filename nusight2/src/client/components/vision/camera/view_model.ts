import { computed } from 'mobx'

import { scene } from '../../three/builders'
import { stage } from '../../three/builders'
import { orthographicCamera } from '../../three/builders'
import { Canvas } from '../../three/three'
import { ImageViewModel } from '../image_view/view_model'

import { BallsViewModel } from './balls'
import { CompassViewModel } from './compass'
import { DistanceViewModel } from './distance'
import { GoalsViewModel } from './goals'
import { GreenHorizonViewModel } from './greenhorizon'
import { HorizonViewModel } from './horizon'
import { CameraModel } from './model'
import { VisualMeshViewModel } from './visual_mesh'

export class CameraViewModel {
  constructor(private readonly canvas: Canvas, private readonly model: CameraModel) {}

  static of(canvas: Canvas, model: CameraModel) {
    return new CameraViewModel(canvas, model)
  }

  readonly stage = stage(() => ({ camera: this.camera(), scene: this.scene() }))

  readonly camera = orthographicCamera(() => ({
    left: -1,
    right: 1,
    top: 1,
    bottom: -1,
    near: 0,
    far: 1,
  }))

  readonly scene = scene(() => {
    const { drawOptions } = this.model
    return {
      children: [
        drawOptions.drawImage && this.image.image(),
        drawOptions.drawVisualmesh && this.visualmesh?.visualmesh(),
        drawOptions.drawDistance && this.distance.distance(),
        drawOptions.drawCompass && this.compass.compass(),
        drawOptions.drawHorizon && this.horizon.horizon(),
        drawOptions.drawGreenhorizon && this.greenhorizon?.greenhorizon(),
        drawOptions.drawBalls && this.balls?.balls(),
        drawOptions.drawGoals && this.goals?.goals(),
      ],
    }
  })

  @computed
  private get compass(): CompassViewModel {
    return CompassViewModel.of(this.canvas, this.model.params)
  }

  @computed
  private get horizon(): HorizonViewModel {
    return HorizonViewModel.of(this.canvas, this.model.params)
  }

  @computed
  private get distance(): DistanceViewModel {
    return DistanceViewModel.of(this.canvas, this.model.params)
  }

  @computed
  private get greenhorizon(): GreenHorizonViewModel | undefined {
    const { greenhorizon, params } = this.model
    return greenhorizon && GreenHorizonViewModel.of(this.canvas, greenhorizon, params)
  }

  @computed
  private get balls(): BallsViewModel | undefined {
    return this.model.balls && BallsViewModel.of(this.model.balls, this.canvas, this.model.params)
  }

  @computed
  private get goals(): GoalsViewModel | undefined {
    return this.model.goals && GoalsViewModel.of(this.model.goals, this.canvas, this.model.params)
  }

  @computed
  private get visualmesh(): VisualMeshViewModel | undefined {
    const { visualmesh, params } = this.model
    return visualmesh && VisualMeshViewModel.of(this.canvas, visualmesh, params)
  }

  @computed
  private get image(): ImageViewModel {
    return ImageViewModel.of(this.model.image)
  }
}
