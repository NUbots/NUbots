import { observable } from "mobx";

export class SkyboxModel {
  @observable accessor turbidity: number;
  @observable accessor rayleigh: number;
  @observable accessor mieCoefficient: number;
  @observable accessor mieDirectionalG: number; // default 0.8
  @observable accessor luminance: number;
  @observable accessor inclination: number; // elevation / inclination
  @observable accessor azimuth: number; // Facing front,
  @observable accessor showSun: boolean;

  constructor({
    turbidity,
    rayleigh,
    mieCoefficient,
    mieDirectionalG,
    luminance,
    inclination,
    azimuth,
    showSun,
  }: SkyboxModel) {
    this.turbidity = turbidity;
    this.rayleigh = rayleigh;
    this.mieCoefficient = mieCoefficient;
    this.mieDirectionalG = mieDirectionalG;
    this.luminance = luminance;
    this.inclination = inclination;
    this.azimuth = azimuth;
    this.showSun = showSun;
  }

  static of() {
    return new SkyboxModel({
      turbidity: 10,
      rayleigh: 2,
      mieCoefficient: 0.005,
      mieDirectionalG: 0.53, // default 0.8
      luminance: 0.5,
      inclination: 0.49, // elevation / inclination
      azimuth: 0.05, // Facing front,
      showSun: false,
    });
  }
}
