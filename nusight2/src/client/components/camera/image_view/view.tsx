import React from "react";
import { Component } from "react";
import { computed } from "mobx";

import { orthographicCamera } from "../../three/builders";
import { stage } from "../../three/builders";
import { scene } from "../../three/builders";
import { ObjectFit } from "../../three/three";
import { Three } from "../../three/three";
import { Image } from "../image";

import { ImageViewModel } from "./view_model";

export class ImageView extends Component<{ image: Image }> {
  render() {
    return <Three stage={this.stages} objectFit={this.objectFit} />;
  }

  @computed
  get objectFit(): ObjectFit {
    return { type: "contain", aspect: this.props.image.height / this.props.image.width };
  }

  private stages = () => computed(() => [this.stage]);

  private readonly stage = stage(() => ({ scene: this.scene(), camera: this.camera() }));

  private readonly scene = scene(() => ({ children: [this.imageView.image()] }));

  @computed
  private get imageView() {
    return ImageViewModel.of(this.props.image);
  }

  private readonly camera = orthographicCamera(() => ({
    left: -1,
    right: 1,
    top: 1,
    bottom: -1,
    near: 0,
    far: 1,
  }));
}
