import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'

import { CameraViewModel } from './camera/view_model'
import { VisualMeshModel } from './model'
import { VisualMeshRobotModel } from './model'

export class VisualMeshViewModel {
  constructor(private model: VisualMeshModel) {}

  static of = createTransformer((model: VisualMeshModel): VisualMeshViewModel => {
    return new VisualMeshViewModel(model)
  })

  @computed
  get robots(): RobotViewModel[] {
    return this.visibleRobots.map(RobotViewModel.of)
  }

  @computed
  private get visibleRobots(): VisualMeshRobotModel[] {
    return this.model.robots.filter(robot => robot.visible && robot.cameras.size > 0)
  }
}

export class RobotViewModel {
  constructor(private model: VisualMeshRobotModel) {}

  static of = createTransformer((model: VisualMeshRobotModel) => {
    return new RobotViewModel(model)
  })

  @computed
  get id() {
    return this.model.id
  }

  @computed
  get name() {
    return this.model.name
  }

  @computed
  get cameras(): CameraViewModel[] {
    return Array.from(this.model.cameras.values(), CameraViewModel.of).sort((a, b) => a.id - b.id)
  }
}
