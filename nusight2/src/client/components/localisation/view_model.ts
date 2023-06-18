import { computed } from "mobx";
import { Color } from "three";
import { Mesh } from "three";
import { HemisphereLight } from "three";
import { PointLight } from "three";
import { Object3D } from "three";

import { Vector3 } from "../../../shared/math/vector3";
import { meshBasicMaterial } from "../three/builders";
import { circleBufferGeometry } from "../three/builders";
import { stage } from "../three/builders";
import { scene } from "../three/builders";
import { perspectiveCamera } from "../three/builders";
import { Canvas } from "../three/three";

import { FieldView } from "./field/view";
import { LocalisationModel } from "./model";
import { NUgusView } from "./nugus_robot/view";
import { SkyboxView } from "./skybox/view";

export class LocalisationViewModel {
  private readonly canvas: Canvas;
  private readonly model: LocalisationModel;

  constructor(canvas: Canvas, model: LocalisationModel) {
    this.canvas = canvas;
    this.model = model;
  }

  static of(canvas: Canvas, model: LocalisationModel) {
    return new LocalisationViewModel(canvas, model);
  }

  readonly stage = stage(() => ({ camera: this.camera(), scene: this.scene() }));

  private readonly camera = perspectiveCamera(() => ({
    fov: 75,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.01,
    far: 100,
    position: this.model.camera.position,
    rotation: Vector3.from({
      x: Math.PI / 2 + this.model.camera.pitch,
      y: 0,
      z: -Math.PI / 2 + this.model.camera.yaw,
    }),
    rotationOrder: "ZXY",
    up: Vector3.from({ x: 0, y: 0, z: 1 }),
  }));

  private readonly scene = scene(() => ({
    children: [...this.robots, this.field, this.skybox, this.hemisphereLight, this.pointLight, this.fieldLineDots],
  }));

  @computed
  private get field() {
    return FieldView.of(this.model.field).field;
  }

  @computed
  private get robots(): Object3D[] {
    return this.model.robots
      .filter((robotModel) => robotModel.visible)
      .map((robotModel) => NUgusView.of(robotModel).robot);
  }

  @computed
  private get hemisphereLight(): HemisphereLight {
    return new HemisphereLight("#fff", "#fff", 0.6);
  }

  @computed
  private get skybox() {
    return SkyboxView.of(this.model.skybox).skybox;
  }

  @computed
  private get pointLight() {
    const light = new PointLight("#fff");
    light.position.set(this.model.camera.position.x, this.model.camera.position.y, this.model.camera.position.z);
    return light;
  }

  @computed
  private get fieldLineDots() {
    const group = new Object3D();
    this.model.robots.forEach((robot) =>
      robot.fieldLinesDots.rPWw.forEach((d) => {
        const mesh = new Mesh(
          LocalisationViewModel.fieldLineDotGeometry(),
          LocalisationViewModel.fieldLineDotMaterial(),
        );
        mesh.position.copy(d.add(new Vector3(0, 0, 0.005)).toThree());
        group.add(mesh);
      }),
    );
    return group;
  }

  private static readonly fieldLineDotGeometry = circleBufferGeometry(() => ({ radius: 0.02, segments: 20 }));

  private static readonly fieldLineDotMaterial = meshBasicMaterial(() => ({ color: new Color("blue") }));
}
