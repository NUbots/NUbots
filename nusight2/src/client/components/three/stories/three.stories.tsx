import { Component } from "react";
import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { action } from "mobx";
import { computed } from "mobx";
import { reaction } from "mobx";
import { observable } from "mobx";
import { disposeOnUnmount } from "mobx-react";
import { createTransformer } from "mobx-utils";
import { now } from "mobx-utils";
import { Color } from "three";
import { BufferGeometry } from "three";
import { PointLight } from "three";
import { BoxGeometry } from "three";
import { Light } from "three";

import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { disposableComputed } from "../../../base/disposable_computed";
import { scene } from "../builders";
import { perspectiveCamera } from "../builders";
import { meshPhongMaterial } from "../builders";
import { mesh } from "../builders";
import { Stage } from "../three";
import { Canvas } from "../three";
import { Three } from "../three";

type StoryComponent = React.FunctionComponent<{}>;

const meta: Meta<StoryComponent> = {
  title: "components/Three",
  parameters: {
    layout: "fullscreen",
  },
};

export default meta;

export const StaticScene: StoryObj<StoryComponent> = {
  name: "static scene",
  render: () => {
    return <BoxVisualiser />;
  },
};

export const AnimatedScene: StoryObj<StoryComponent> = {
  name: "animated scene",
  render: () => {
    return <BoxVisualiser animate />;
  },
};

type Model = { boxes: BoxModel[] };
type BoxModel = { color: string; size: number; position: Vector3; rotation: Vector3 };

class BoxVisualiser extends Component<{ animate?: boolean }> {
  @observable
  private readonly model = {
    boxes: [
      { color: "red", size: 1, position: Vector3.of(), rotation: Vector3.of() },
      { color: "green", size: 1, position: Vector3.of(), rotation: Vector3.of() },
      { color: "blue", size: 1, position: Vector3.of(), rotation: Vector3.of() },
    ],
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
    return <Three stage={this.stage} />;
  }

  private stage = (canvas: Canvas) => {
    const viewModel = new ViewModel(canvas, this.model);
    return computed(() => viewModel.stage);
  };

  @action.bound
  private update(now: number) {
    const t = (2 * Math.PI * now) / (20 * 1000);
    const n = this.model.boxes.length;
    this.model.boxes.forEach((box, i) => {
      const position = Vector2.fromPolar(1, (i * 2 * Math.PI) / n + t);
      box.position = new Vector3(position.x, position.y, 0);
      box.rotation = new Vector3(Math.cos(3 * t + i), Math.cos(5 * t + i), Math.cos(7 * t + i));
    });
  }
}

class ViewModel {
  private readonly canvas: Canvas;
  private readonly model: Model;

  constructor(canvas: Canvas, model: Model) {
    this.canvas = canvas;
    this.model = model;
  }

  @computed
  get stage(): Stage {
    return { camera: this.camera(), scene: this.scene() };
  }

  @computed
  private get light(): Light {
    const light = new PointLight();
    light.position.copy(this.camera().position);
    return light;
  }

  private camera = perspectiveCamera(() => ({
    fov: 60,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.5,
    far: 10,
    position: Vector3.from({ x: 0, y: 0, z: 4 }),
  }));

  private scene = scene(() => ({
    children: [...this.boxes.map((boxViewModel) => boxViewModel.box()), this.light],
  }));

  @computed
  private get boxes() {
    return this.model.boxes.map(ViewModel.getBox);
  }

  private static getBox = createTransformer((box: BoxModel): BoxViewModel => {
    return BoxViewModel.of(box);
  });
}

class BoxViewModel {
  private readonly model: BoxModel;

  private static geometry = disposableComputed<BufferGeometry>(() => new BoxGeometry(1, 1, 1));

  constructor(model: BoxModel) {
    this.model = model;
  }

  static of(model: BoxModel): BoxViewModel {
    return new BoxViewModel(model);
  }

  readonly box = mesh(() => ({
    geometry: BoxViewModel.geometry.get(),
    material: this.material(),
    position: this.model.position,
    rotation: this.model.rotation,
    scale: Vector3.fromScalar(this.model.size),
  }));

  // @disposableComputed
  private material = meshPhongMaterial(() => ({
    color: new Color(this.model.color),
  }));
}
