import React from "react";
import { Component } from "react";
import classNames from "classnames";
import { IReactionDisposer } from "mobx";
import { observable } from "mobx";
import { action } from "mobx";
import { autorun } from "mobx";
import { observer } from "mobx-react";
import ReactResizeDetector from "react-resize-detector";

import { Transform } from "../../shared/math/transform";
import { Vector2 } from "../../shared/math/vector2";

import { renderObject2d } from "./canvas/rendering";
import { RendererProps } from "./renderer";
import style from "./style.module.css";

@observer
export class CanvasRenderer extends Component<RendererProps> {
  @observable private resolution: Transform = Transform.of();
  private canvas: HTMLCanvasElement | null = null;
  private stopAutorun?: IReactionDisposer;

  componentDidMount() {
    // Render when changes happen
    this.stopAutorun = autorun(this.renderCanvas, {
      scheduler: requestAnimationFrame,
    });
  }

  componentWillUnmount() {
    if (this.stopAutorun) {
      this.stopAutorun();
    }
  }

  render() {
    return (
      <div className={classNames(this.props.className, style.container)}>
        <ReactResizeDetector handleWidth handleHeight onResize={this.onResize} />
        <canvas
          className={style.container}
          width={Math.abs(this.resolution.translate.x * 2)}
          height={Math.abs(this.resolution.translate.y * 2)}
          ref={this.onRef}
        />
      </div>
    );
  }

  private onRef = (canvas: HTMLCanvasElement) => {
    this.canvas = canvas;
  };

  renderCanvas = () => {
    // Render our scene
    const { scene, camera } = this.props;

    const cam = this.resolution.then(camera.inverse());
    const ctx = this.canvas!.getContext("2d")!;

    ctx.save();
    ctx.clearRect(0, 0, this.canvas!.width, this.canvas!.height);
    renderObject2d(ctx, scene, cam);
    ctx.restore();
  };

  @action
  private onResize = (width: number, height: number) => {
    // Multiply all our widths by dpi
    width *= devicePixelRatio;
    height *= devicePixelRatio;

    const translate = Vector2.of(width * 0.5, height * 0.5);

    // If we have an aspect ratio, use it to scale the canvas to unit size
    // The scale is flipped here so that the coordinate system has positive y up
    if (this.props.aspectRatio !== undefined) {
      const canvasAspect = width / height;
      const scale = canvasAspect < this.props.aspectRatio ? width : height * this.props.aspectRatio;
      // Scale to fit
      this.resolution = Transform.of({ scale: { x: scale, y: -scale }, translate });
    } else {
      this.resolution = Transform.of({ scale: { x: width, y: -height }, translate });
    }

    this.props.onResize?.(width, height);
  };
}
