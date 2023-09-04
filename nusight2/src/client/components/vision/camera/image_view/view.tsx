import React from "react";
import { Component } from "react";
import { computed } from "mobx";

import { ObjectFit } from "../../../three/three";
import { OrthographicCamera } from "../../../three/three_fiber";
import { ThreeFiber } from "../../../three/three_fiber";
import { Image } from "../image";

import { ImageView as VisionImageView } from "./view_model";

export class ImageView extends Component<{ image: Image }> {
  render() {
    return (
      <ThreeFiber objectFit={this.objectFit}>
        <OrthographicCamera args={[-1, 1, 1, -1, 0, 1]} manual />
        <VisionImageView image={this.props.image} />
      </ThreeFiber>
    );
  }

  @computed
  get objectFit(): ObjectFit {
    return { type: "contain", aspect: this.props.image.height / this.props.image.width };
  }
}
