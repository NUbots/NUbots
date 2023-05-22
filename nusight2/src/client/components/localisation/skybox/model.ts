import { observable } from "mobx";

export class SkyboxModel {
  @observable turbidity: number;
  @observable rayleigh: number;
  @observable mieCoefficient: number;
  @observable mieDirectionalG: number; // default 0.8
  @observable luminance: number;
  @observable inclination: number; // elevation / inclination
  @observable azimuth: number; // Facing front,
  @observable showSun: boolean;

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
      azimuth: 0.25, // Facing front,
      showSun: false,
    });
  }
}
