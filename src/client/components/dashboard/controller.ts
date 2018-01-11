import { action } from 'mobx'
import { DashboardModel } from './model'

export class DashboardController {
  public static of() {
    return new DashboardController()
  }

  @action
  public toggleOrientation(model: DashboardModel) {
    model.field.orientation = model.field.orientation === 'left' ? 'right' : 'left'
  }
}
