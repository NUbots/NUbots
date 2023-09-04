import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import classNames from "classnames";
import { observable } from "mobx";
import { action } from "mobx";
import { computed } from "mobx";
import { observer } from "mobx-react";
import { Object3D } from "three";
import { Event } from "three";

import { Vector2 } from "../../../../shared/math/vector2";
import { SwitchesMenuOption } from "../../switches_menu/view";
import { SwitchesMenu } from "../../switches_menu/view";
import { ObjectFit } from "../../three/three";
import { OrthographicCamera } from "../../three/three_fiber";
import { ThreeFiber } from "../../three/three_fiber";
import { VisionRobotModel } from "../model";

import { IconZoomIn, IconZoomOut, IconZoomReset } from "./icons";
import { ImageView } from "./image_view/view_model";
import { VisionCameraModel } from "./model";
import { BallsView } from "./objects/balls";
import { CompassView } from "./objects/compass";
import { DistanceView } from "./objects/distance";
import { GoalsView } from "./objects/goals";
import { GreenHorizonView } from "./objects/green_horizon";
import { HorizonView } from "./objects/horizon";
import { VisualMeshView } from "./objects/visual_mesh";

export type Renderable = false | Object3D<Event> | undefined;

export interface VisionCameraViewProps {
  model: VisionCameraModel;
  robot: VisionRobotModel;
  viewType: "full" | "thumbnail";
  objectFit: "fill" | "contain" | "cover";
  allowPanAndZoom: boolean;
  onClick?({ button }: { button: number }): void;
}

/**
 * Base Camera View Renderer.
 *
 * This is responsible for drawing the basic camera objects:
 *  - Image
 *  - Distance indicator
 *  - Compass
 *  - Horizon line
 *
 * and basic controls for:
 *  - Zooming
 *  - Panning
 *
 * Extra renderables can be added through this Component's props as well
 * as extra options for the switch menu.
 */
@observer
export class VisionCameraView extends Component<VisionCameraViewProps> {
  static readonly minZoom = 1;
  static readonly maxZoom = 10;

  @observable lastMousePosition?: Vector2;

  @observable canvas = React.createRef<HTMLCanvasElement>();

  static defaultProps = {
    viewType: "full",
    objectFit: "contain",
    allowPanAndZoom: false,
  } as const;

  render() {
    const { model, viewType, allowPanAndZoom, onClick } = this.props;
    return (
      <div
        className={classNames("relative w-full h-full", {
          "bg-black": viewType === "full",
          "bg-transparent": viewType === "thumbnail",
        })}
      >
        {allowPanAndZoom ? (
          <ThreeFiber
            ref={this.canvas}
            objectFit={this.objectFit}
            onClick={onClick}
            onWheel={(e) => this.zoomScroll(e.deltaY)}
            onMouseDown={this.startPan}
            onMouseUp={this.endPan}
            onMouseMove={(e) => this.onPan((e.nativeEvent as any).layerX, (e.nativeEvent as any).layerY)}
          >
            {this.renderCamera()}
          </ThreeFiber>
        ) : (
          <ThreeFiber ref={this.canvas} objectFit={this.objectFit} onClick={onClick}>
            {this.renderCamera()}
          </ThreeFiber>
        )}
        {viewType === "full" ? (
          <div className="flex flex-col absolute top-0 right-0">
            <SwitchesMenu dropdownMenuPosition="right" options={this.drawOptions} />
            {allowPanAndZoom ? (
              <div className="flex flex-col">
                <IconButton Icon={IconZoomIn} title="Zoom In" onClick={() => this.zoomCenter(1)} />
                <IconButton Icon={IconZoomOut} title="Zoom Out" onClick={() => this.zoomCenter(-1)} />
                <IconButton Icon={IconZoomReset} title="Zoom Reset" onClick={this.resetCamera} />
              </div>
            ) : null}
          </div>
        ) : null}
        <div
          className={classNames("absolute bg-[rgba(0,0,0,0.6)] text-white text-sm leading-none", {
            "top-0 left-0 rounded-br px-2 pt-2 pb-[6px] text-sm": viewType === "thumbnail",
            "top-2 left-[50%] translate-x-[-50%] rounded p-2": viewType === "full",
          })}
        >
          {model.name}
        </div>
      </div>
    );
  }

  renderCamera() {
    const { image, params, greenHorizon, goals, balls, visualMesh, drawOptions: opts } = this.props.model;
    const imageAspectRatio = image.width / image.height;
    return (
      <>
        <OrthographicCamera args={[-1, 1, 1, -1, 0, 1]} manual />
        {opts.drawImage && <ImageView image={image} />}
        {opts.drawDistance && <DistanceView params={params} imageAspectRatio={imageAspectRatio} />}
        {opts.drawCompass && <CompassView params={params} imageAspectRatio={imageAspectRatio} />}
        {opts.drawHorizon && <HorizonView params={params} imageAspectRatio={imageAspectRatio} />}
        {opts.drawVisualMesh && visualMesh && (
          <VisualMeshView model={visualMesh} params={params} imageAspectRatio={imageAspectRatio} />
        )}
        {opts.drawGreenHorizon && greenHorizon && (
          <GreenHorizonView model={greenHorizon} params={params} imageAspectRatio={imageAspectRatio} />
        )}
        {opts.drawBalls && balls && <BallsView model={balls} params={params} imageAspectRatio={imageAspectRatio} />}
        {opts.drawGoals && goals && <GoalsView model={goals} params={params} imageAspectRatio={imageAspectRatio} />}
      </>
    );
  }

  @computed
  private get drawOptions(): SwitchesMenuOption[] {
    const { drawOptions } = this.props.model;
    return [
      {
        label: "Image",
        enabled: drawOptions.drawImage,
        toggle: action(() => (drawOptions.drawImage = !drawOptions.drawImage)),
      },
      {
        label: "Distance",
        enabled: drawOptions.drawDistance,
        toggle: action(() => (drawOptions.drawDistance = !drawOptions.drawDistance)),
      },
      {
        label: "Compass",
        enabled: drawOptions.drawCompass,
        toggle: action(() => (drawOptions.drawCompass = !drawOptions.drawCompass)),
      },
      {
        label: "Horizon",
        enabled: drawOptions.drawHorizon,
        toggle: action(() => (drawOptions.drawHorizon = !drawOptions.drawHorizon)),
      },
      {
        label: "Visual Mesh",
        enabled: drawOptions.drawVisualMesh,
        toggle: action(() => (drawOptions.drawVisualMesh = !drawOptions.drawVisualMesh)),
      },
      {
        label: "Green Horizon",
        enabled: drawOptions.drawGreenHorizon,
        toggle: action(() => (drawOptions.drawGreenHorizon = !drawOptions.drawGreenHorizon)),
      },
      {
        label: "Balls",
        enabled: drawOptions.drawBalls,
        toggle: action(() => (drawOptions.drawBalls = !drawOptions.drawBalls)),
      },
      {
        label: "Goals",
        enabled: drawOptions.drawGoals,
        toggle: action(() => (drawOptions.drawGoals = !drawOptions.drawGoals)),
      },
    ];
  }

  @computed.struct
  private get objectFit(): ObjectFit {
    switch (this.props.objectFit) {
      case "fill":
        return { type: "fill" };
      case "contain":
      case "cover":
        return { type: this.props.objectFit, aspect: this.props.model.image.width / this.props.model.image.height };
    }
  }

  zoomScroll = (deltaY: number) => {
    const { width, height } = this.canvas.current!;

    // Mouse pixel offset from the centre of the canvas
    const lastMouse = this.lastMousePosition ?? new Vector2(0, 0);
    const centerOffset = lastMouse.subtract(new Vector2(width / 2, height / 2));

    this.onZoom(centerOffset, -Math.sign(deltaY) * 0.25);
  };

  zoomCenter = (zoomAmount: number) => {
    this.onZoom(Vector2.of(0, 0), zoomAmount);
  };

  @action
  onZoom = (centerOffset: Vector2, zoomAmount: number) => {
    // Pixel offset of the mouse from the centre of the image if the zoom level was 1 (no zoom)
    // Used to zoom in/out while maintaining focus on the mouse position
    const scalelessOffset = this.props.model.pan.add(centerOffset).divideScalar(this.props.model.zoom);

    // Set the new pan/zoom
    this.props.model.zoom = this.limitZoom(this.props.model.zoom + zoomAmount);

    const newPan = scalelessOffset.multiplyScalar(this.props.model.zoom).subtract(centerOffset);
    this.props.model.pan = this.limitPan(newPan);
  };

  @action
  startPan = () => {
    this.props.model.isPanning = true;
  };

  @action
  endPan = () => {
    this.props.model.isPanning = false;
  };

  @action
  onPan = (x: number, y: number) => {
    const mousePos = new Vector2(x, y);

    // Only pan while left mouse is held
    if (this.props.model.isPanning && this.lastMousePosition !== undefined) {
      const displacement = this.lastMousePosition.subtract(mousePos);
      this.props.model.pan = this.limitPan(this.props.model.pan.add(displacement));
    }

    // Track the mouse position
    this.lastMousePosition = mousePos;
  };

  @action
  resetCamera = () => {
    this.props.model.zoom = 1;
    this.props.model.pan = new Vector2(0, 0);
  };

  private limitZoom = (zoom: number) => {
    return Math.min(Math.max(zoom, VisionCameraView.minZoom), VisionCameraView.maxZoom);
  };

  private limitPan = (pan: Vector2) => {
    const zoom = this.props.model.zoom;
    const widthLimit = (this.canvas.current!.width / 2) * (zoom - 1);
    const heightLimit = (this.canvas.current!.height / 2) * (zoom - 1);

    // Apply min/max values for pan
    const x = Math.min(widthLimit, Math.max(pan.x, -widthLimit));
    const y = Math.min(heightLimit, Math.max(pan.y, -heightLimit));

    return new Vector2(x, y);
  };
}

interface IconButtonProps extends React.HTMLAttributes<HTMLButtonElement> {
  Icon: ComponentType<{ className: string }>;
}
const IconButton = (props: IconButtonProps) => {
  const { Icon, ...otherProps } = props;
  return (
    <button className="hover:bg-white h-12 transition-colors duration-300" {...otherProps}>
      <Icon className="w-7 h-7 m-auto" />
    </button>
  );
};
