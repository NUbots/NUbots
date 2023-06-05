import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { observer } from "mobx-react";

import { CameraView } from "./camera/view";
import { VisualMeshNetwork } from "./network";
import styles from "./style.module.css";
import { VisualMeshViewModel } from "./view_model";

@observer
export class VisualMeshView extends Component<{
  viewModel: VisualMeshViewModel;
  network: VisualMeshNetwork;
  Menu: ComponentType;
}> {
  componentWillUnmount() {
    this.props.network.destroy();
  }

  render() {
    const {
      viewModel: { robots },
      Menu,
    } = this.props;
    return (
      <div className={styles.vision}>
        <Menu />
        {robots.map(({ id, name, cameras }) => (
          <div key={id}>
            <h1>{name}</h1>
            <div className={styles.cameras}>
              {cameras.map((camera) => (
                <CameraView key={camera.id} viewModel={camera} />
              ))}
            </div>
          </div>
        ))}
      </div>
    );
  }
}
