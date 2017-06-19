import { observable } from 'mobx'

export class SkyboxModel {

  @observable public turbidity: number
  @observable public rayleigh: number
  @observable public mieCoefficient: number
  @observable public mieDirectionalG: number // default 0.8
  @observable public luminance: number
  @observable public inclination: number // elevation / inclination
  @observable public azimuth: number // Facing front,
  @observable public showSun: boolean

  public constructor(opts: SkyboxModel) {
    Object.assign(this, opts)
  }

  public static of() {
    return new SkyboxModel({
      turbidity: 10,
      rayleigh: 2,
      mieCoefficient: 0.005,
      mieDirectionalG: 0.53, // default 0.8
      luminance: 0.5,
      inclination: 0.49, // elevation / inclination
      azimuth: 0.25, // Facing front,
      showSun: false,
    })
  }
}
