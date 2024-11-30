import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { reaction } from "mobx";
import { now } from "mobx-utils";
import * as THREE from "three";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector3 } from "../../../../../shared/math/vector3";
import { OdometryVisualizerModel } from "../model";
import { OdometryVisualizer } from "../view";

interface StoryProps {}

const meta: Meta<StoryProps> = {
  title: "components/odometry/odometry_visualizer",
  parameters: { layout: "fullscreen" },
};

export default meta;

export const Static: StoryObj<StoryProps> = {
  render: () => <OdometryVisualizerHarness />,
};

export const Animated: StoryObj<StoryProps> = {
  render: () => <OdometryVisualizerHarness animate />,
};

class OdometryVisualizerHarness extends React.Component<{ animate?: boolean }> {
  private dispose?: () => void;

  private model = OdometryVisualizerModel.of({
    Hwt: Matrix4.fromThree(new THREE.Matrix4().makeTranslation(0, 0, 1)),
    accelerometer: new Vector3(0, 0, -9.8),
  });

  componentDidMount() {
    this.dispose = this.props.animate
      ? reaction(
          () => now("frame") / 1000,
          (t) => {
            this.model.Hwt = Matrix4.fromThree(
              new THREE.Matrix4()
                .makeRotationFromEuler(new THREE.Euler(Math.cos(t) / 5, 0, t))
                .multiply(
                  new THREE.Matrix4().makeTranslation(
                    Math.cos((7 * t) / 10),
                    Math.cos((5 * t) / 10),
                    Math.cos((3 * t) / 10) + 2,
                  ),
                ),
            );
          },
        )
      : undefined;
  }

  componentWillUnmount() {
    this.dispose?.();
    this.dispose = undefined;
  }

  render() {
    return <OdometryVisualizer model={this.model} />;
  }
}
