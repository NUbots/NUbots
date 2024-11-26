import { WheelEvent } from "react";
import { MouseEvent } from "react";
import React from "react";
import { Component } from "react";
import { reaction } from "mobx";
import { IComputedValue } from "mobx";
import { observable } from "mobx";
import { action } from "mobx";
import { autorun } from "mobx";
import { disposeOnUnmount } from "mobx-react";
import { observer } from "mobx-react";
import { ContentRect } from "react-measure";
import Measure from "react-measure";
import { Color } from "three";
import { WebGLRenderTarget } from "three";
import { WebGLRenderer } from "three";
import { Scene } from "three";
import { Camera } from "three";
import { throttle } from "throttle-debounce";

import { compose } from "../../../shared/base/compose";

import style from "./style.module.css";

export type Stage = { scene: Scene; camera: Camera; target?: WebGLRenderTarget };
export type Canvas = { width: number; height: number };

@observer
export class Three extends Component<{
  stage(canvas: Canvas): IComputedValue<Stage | (() => Stage)[]>;
  objectFit: ObjectFit;
  clearColor?: Color;
  onClick?({ button }: { button: number }): void;
  onMouseDown?(x: number, y: number): void;
  onMouseMove?(x: number, y: number): void;
  onMouseUp?(x: number, y: number): void;
  onWheel?(deltaY: number, preventDefault: () => void): void;
  renderScheduler?: (callback: () => void) => any;
}> {
  @observable private accessor containerSize = { width: 0, height: 0 };
  @observable private accessor canvas: Canvas = { width: 0, height: 0 };
  private ref: HTMLCanvasElement | null = null;
  private renderer?: WebGLRenderer;

  static defaultProps = {
    objectFit: { type: "fill" },
  };

  componentDidMount() {
    this.renderer = new WebGLRenderer({ canvas: this.ref!, antialias: true });
    this.props.clearColor && this.renderer.setClearColor(this.props.clearColor);
    const stages = this.props.stage!(this.canvas);
    // TODO (Annable): Extract this and add unit tests.
    let dispose: () => void = () => undefined;
    disposeOnUnmount(this, [
      reaction(
        () => stages.get(),
        (stages) => {
          dispose();
          if (stages instanceof Array) {
            // Create individual reactions for each stage, so they may react and re-render independently.
            dispose = compose(
              stages.map((stage) =>
                autorun(() => this.renderStage(stage()), {
                  scheduler: this.props.renderScheduler ?? requestAnimationFrame,
                }),
              ),
            );
          } else {
            dispose = autorun(() => this.renderStage(stages), {
              scheduler: this.props.renderScheduler ?? requestAnimationFrame,
            });
          }
          disposeOnUnmount(this, dispose);
        },
        { fireImmediately: true },
      ),
      reaction(
        () => objectFit(this.containerSize, this.props.objectFit),
        throttle(
          100,
          action(({ width, height }: { width: number; height: number }) => {
            this.canvas.width = width;
            this.canvas.height = height;
          }),
        ),
        { fireImmediately: true },
      ),
    ]);
  }

  componentWillUnmount() {
    this.renderer?.forceContextLoss();
    this.renderer?.dispose();
    delete this.renderer;
  }

  render() {
    const { objectFit } = this.props;
    return (
      <Measure bounds onResize={this.onResize} innerRef={this.setRef}>
        {({ measureRef }) => (
          <canvas
            ref={measureRef}
            // Use 'none' instead of fill to avoid stretching the visuals during a resize.
            style={{ objectFit: objectFit.type === "fill" ? "none" : objectFit.type }}
            className={style.canvas}
            onClick={this.props.onClick}
            onMouseDown={this.onMouseDown}
            onMouseMove={this.onMouseMove}
            onMouseUp={this.onMouseUp}
            onWheel={this.onWheel}
            onMouseLeave={this.onMouseLeave}
          />
        )}
      </Measure>
    );
  }

  requestPointerLock() {
    this.ref!.requestPointerLock();
  }

  isPointerLocked() {
    return document.pointerLockElement === this.ref!;
  }

  private renderStage(stage: Stage) {
    if (!this.renderer) {
      throw new Error();
    }
    this.renderer.setSize(this.canvas.width, this.canvas.height, false);
    this.renderer.setRenderTarget(stage.target || null);
    this.renderer.render(stage.scene, stage.camera);
    this.renderer.setRenderTarget(null);
  }

  @action.bound
  private onResize(canvas: ContentRect) {
    this.containerSize = { width: canvas.bounds!.width, height: canvas.bounds!.height };
  }

  private readonly setRef = (ref: HTMLCanvasElement | null) => {
    this.ref = ref;
  };

  private onMouseDown = (e: MouseEvent<HTMLCanvasElement>) => {
    // TODO: Remove deprecated layerX
    this.props.onMouseDown && this.props.onMouseDown((e.nativeEvent as any).layerX, (e.nativeEvent as any).layerY);
  };

  private onMouseMove = (e: MouseEvent<HTMLCanvasElement>) => {
    // TODO: Remove deprecated layerX
    this.props.onMouseMove && this.props.onMouseMove((e.nativeEvent as any).layerX, (e.nativeEvent as any).layerY);
  };

  private onMouseUp = (e: MouseEvent<HTMLCanvasElement>) => {
    // TODO: Remove deprecated layerX
    this.props.onMouseUp && this.props.onMouseUp((e.nativeEvent as any).layerX, (e.nativeEvent as any).layerY);
  };

  // Prevent dragging actions from becoming stuck when dragging off the canvas
  private onMouseLeave = (e: MouseEvent<HTMLCanvasElement>) => {
    this.onMouseUp(e);
  };

  private onWheel = (e: WheelEvent<HTMLCanvasElement>) => {
    this.props.onWheel && this.props.onWheel(e.deltaY, () => e.preventDefault());
  };
}

// Based on the object-fit CSS property: https://developer.mozilla.org/en-US/docs/Web/CSS/object-fit
export type ObjectFit =
  // Stretch content to fill entire container.
  | { type: "fill" }
  // Either cover the container with content, or contain the content in the container, while maintaining aspect ratio.
  | { type: "contain" | "cover"; aspect: number };

export function objectFit(
  container: { width: number; height: number },
  objectFit: ObjectFit,
): { width: number; height: number } {
  switch (objectFit.type) {
    case "fill":
      return { width: container.width, height: container.height };
    case "contain":
    case "cover": {
      const containerAspect = container.height / container.width;
      const ratio = containerAspect / objectFit.aspect;
      if (objectFit.type === "contain" ? ratio >= 1 : ratio < 1) {
        // Take entire container width, scale the height
        return { width: container.width, height: Math.ceil(container.width * objectFit.aspect) };
      } else {
        // Take entire container height, scale the width
        return { width: Math.ceil(container.height / objectFit.aspect), height: container.height };
      }
    }
    default:
      throw new Error(`unknown object fit mode: ${objectFit}`);
  }
}
