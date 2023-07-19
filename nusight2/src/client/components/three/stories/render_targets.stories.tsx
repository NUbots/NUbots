import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { action } from "mobx";
import { reaction } from "mobx";
import { computed } from "mobx";
import { observable } from "mobx";
import { disposeOnUnmount } from "mobx-react";
import { now } from "mobx-utils";
import { WebGLRenderTarget } from "three";
import { TextureLoader } from "three";
import { LinearFilter } from "three";
import { ClampToEdgeWrapping } from "three";
import { Texture } from "three";
import { Color } from "three";

import { Vector3 } from "../../../../shared/math/vector3";
import { disposableComputed } from "../../../base/disposable_computed";
import { planeGeometry } from "../builders";
import { pointLight } from "../builders";
import { ambientLight } from "../builders";
import { boxGeometry } from "../builders";
import { stage } from "../builders";
import { meshPhongMaterial } from "../builders";
import { perspectiveCamera } from "../builders";
import { renderTarget } from "../builders";
import { meshBasicMaterial } from "../builders";
import { mesh } from "../builders";
import { scene } from "../builders";
import { orthographicCamera } from "../builders";
import { Canvas } from "../three";
import { Three } from "../three";

import robotSvgUrl from "./robot.svg?url";

type StoryComponent = React.FunctionComponent<{}>;

const meta: Meta<StoryComponent> = {
  title: "components/Three",
  parameters: {
    layout: "fullscreen",
  },
};

export default meta;

export const StaticSceneWithRenderTargets: StoryObj<StoryComponent> = {
  name: "static scene with render targets",
  render: () => {
    return <RenderTargetHarness />;
  },
};

export const AnimatedSceneWithRenderTargets: StoryObj<StoryComponent> = {
  name: "animated scene with render targets",
  render: () => {
    return <RenderTargetHarness animate />;
  },
};

type Model = { time: number };

class RenderTargetHarness extends React.Component<{ animate?: boolean }> {
  @observable
  private readonly model: Model = {
    time: 0,
  };

  componentDidMount() {
    this.update(0);
    this.props.animate &&
      disposeOnUnmount(
        this,
        reaction(() => now("frame"), this.update),
      );
  }

  render() {
    return <Three stage={this.stage} clearColor={new Color("white")} />;
  }

  private stage = (canvas: Canvas) => {
    const robotRenderTarget = renderTarget(() => ({ width: 512, height: 512 }));
    const robotTexture = () => robotRenderTarget().texture;
    const robotViewModel = RobotViewModel.of(this.model, robotRenderTarget);
    const innerBoxRenderTarget = renderTarget(() => ({ width: 512, height: 512 }));
    const innerBoxTexture = () => innerBoxRenderTarget().texture;
    const innerBoxViewModel = OrangeBoxViewModel.of(this.model, robotTexture, innerBoxRenderTarget);
    const viewModel = WhiteBoxViewModel.of(canvas, this.model, innerBoxTexture);
    return computed(() => [robotViewModel.stage, innerBoxViewModel.stage, viewModel.stage]);
  };

  @action.bound
  private update(now: number) {
    this.model.time = (2 * Math.PI * now) / (60 * 1000);
  }
}

class WhiteBoxViewModel {
  private readonly canvas: Canvas;
  private readonly model: Model;
  private readonly orangeBoxTexture: () => Texture;

  constructor(canvas: Canvas, model: Model, orangeBoxTexture: () => Texture) {
    this.canvas = canvas;
    this.model = model;
    this.orangeBoxTexture = orangeBoxTexture;
  }

  static of(canvas: Canvas, model: Model, orangeBoxTexture: () => Texture) {
    return new WhiteBoxViewModel(canvas, model, orangeBoxTexture);
  }

  readonly stage = stage(() => ({
    camera: this.camera(),
    scene: this.scene(),
  }));

  private readonly camera = perspectiveCamera(() => ({
    fov: 60,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.5,
    far: 10,
    position: this.cameraPosition,
  }));

  @computed
  private get cameraPosition() {
    return Vector3.from({ x: 0, y: 0, z: 2 });
  }

  private readonly scene = scene(() => ({
    children: [this.ambientLight(), this.pointLight(), this.box()],
  }));

  private readonly ambientLight = ambientLight(() => ({ intensity: 0.5 }));

  private readonly pointLight = pointLight(() => ({
    intensity: 0.5,
    position: this.cameraPosition,
  }));

  private readonly box = mesh(() => ({
    geometry: this.geometry(),
    material: this.material(),
    rotation: new Vector3(3 * this.model.time, 5 * this.model.time, 7 * this.model.time),
  }));

  private readonly geometry = boxGeometry(() => ({ width: 1, height: 1, depth: 1 }));

  private readonly material = meshPhongMaterial(() => ({ map: this.orangeBoxTexture() }));
}

class OrangeBoxViewModel {
  private readonly model: Model;
  private readonly robotTexture: () => Texture;
  private readonly renderTarget: () => WebGLRenderTarget;

  constructor(model: Model, robotTexture: () => Texture, renderTarget: () => WebGLRenderTarget) {
    this.model = model;
    this.robotTexture = robotTexture;
    this.renderTarget = renderTarget;
  }

  static of(model: Model, robotTexture: () => Texture, renderTarget: () => WebGLRenderTarget) {
    return new OrangeBoxViewModel(model, robotTexture, renderTarget);
  }

  readonly stage = stage(() => ({
    scene: this.scene(),
    camera: this.camera(),
    target: this.renderTarget(),
  }));

  private readonly scene = scene(() => ({
    children: [this.ambientLight(), this.pointLight(), this.box()],
  }));

  private readonly ambientLight = ambientLight(() => ({
    color: new Color("orange"),
    intensity: 0.5,
  }));

  private readonly pointLight = pointLight(() => ({
    color: new Color("orange"),
    intensity: 0.5,
    position: this.cameraPosition,
  }));

  private readonly camera = perspectiveCamera(() => ({
    fov: 60,
    aspect: 1,
    near: 0.5,
    far: 10,
    position: this.cameraPosition,
  }));

  @computed
  private get cameraPosition() {
    return Vector3.from({ x: 0, y: 0, z: 4 });
  }

  private readonly box = mesh(() => ({
    geometry: this.geometry(),
    material: this.material(),
    rotation: new Vector3(-17 * this.model.time, -13 * this.model.time, -11 * this.model.time),
  }));

  private readonly geometry = boxGeometry(() => ({ width: 2, height: 2, depth: 2 }));

  private readonly material = meshPhongMaterial(() => ({ map: this.robotTexture() }));
}

class RobotViewModel {
  private readonly model: Model;
  private readonly renderTarget: () => WebGLRenderTarget;

  constructor(model: Model, renderTarget: () => WebGLRenderTarget) {
    this.model = model;
    this.renderTarget = renderTarget;
  }

  static of(model: Model, renderTarget: () => WebGLRenderTarget) {
    return new RobotViewModel(model, renderTarget);
  }

  readonly stage = stage(() => ({
    scene: this.scene(),
    camera: this.camera(),
    target: this.renderTarget(),
  }));

  private readonly scene = scene(() => ({ children: [this.robot()] }));

  private readonly camera = orthographicCamera(() => ({
    left: -1,
    right: 1,
    top: 1,
    bottom: -1,
    near: 0,
    far: 1,
  }));

  private readonly robot = mesh(() => ({
    geometry: this.geometry(),
    material: this.material(),
    rotation: new Vector3(0, 0, 17 * this.model.time),
  }));

  private readonly geometry = planeGeometry(() => ({ width: 1, height: 1 }));

  private readonly material = meshBasicMaterial(() => ({
    map: this.texture.get(),
    transparent: true,
  }));

  private readonly texture = disposableComputed<Texture>(() => {
    const texture = new TextureLoader().load(robotSvgUrl);
    texture.generateMipmaps = false;
    texture.wrapS = texture.wrapT = ClampToEdgeWrapping;
    texture.minFilter = LinearFilter;
    texture.magFilter = LinearFilter;
    return texture;
  });
}
