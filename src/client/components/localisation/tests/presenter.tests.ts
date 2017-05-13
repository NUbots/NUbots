import { LocalisationModel } from '../model'
import { ViewMode } from '../model'
import { LocalisationPresenter } from '../presenter'

describe('LocalisationPresenter', () => {
  let presenter: LocalisationPresenter
  let model: LocalisationModel

  beforeEach(() => {
    model = LocalisationModel.of()
    presenter = new LocalisationPresenter({ model })
  })

  describe('clicking the hawk eye button', () => {
    beforeEach(() => {
      presenter.onHawkEyeClick()
    })

    it('resets yaw to 0', () => {
      expect(model.camera.yaw).toEqual(0)
    })

    it('resets pitch to looking vertically down', () => {
      expect(model.camera.pitch).toEqual(-Math.PI / 2)
    })

    it('moves camera above the field', () => {
      expect(model.camera.position).toEqual({ x: 0, y: 5, z: 0 })
    })

    it('resets viewing mode to no clip', () => {
      expect(model.viewMode).toEqual(ViewMode.NO_CLIP)
    })
  })
})
