import React from "react";
import { Component } from "react";
import { autorun } from "mobx";
import { action } from "mobx";
import { observer } from "mobx-react";
import ReactResizeDetector from "react-resize-detector";

import styles from "./style.module.css";
import { CameraViewModel } from "./view_model";

@observer
export class CameraView extends Component<{ viewModel: CameraViewModel }> {
  private destroy: () => void = () => {};

  componentDidMount() {
    this.destroy = autorun(this.renderScene, { scheduler: requestAnimationFrame });
  }

  componentWillUnmount() {
    this.destroy();
  }

  render() {
    return (
      <div className={styles.viewport}>
        <ReactResizeDetector handleWidth handleHeight onResize={this.onResize} />
        <canvas ref={this.onRef} />
      </div>
    );
  }

  @action
  private onRef = (canvas: HTMLCanvasElement | null) => {
    this.props.viewModel.canvas = canvas;
  };

  @action
  private onResize = (width: number, height: number) => {
    const { renderer, canvas } = this.props.viewModel;
    canvas && renderer(canvas)!.setSize(width, height);
  };

  private renderScene = () => {
    const { viewModel } = this.props;
    const renderer = viewModel.renderer(viewModel.canvas);
    if (renderer) {
      renderer.render(viewModel.getScene(), viewModel.camera);
    }
  };
}
