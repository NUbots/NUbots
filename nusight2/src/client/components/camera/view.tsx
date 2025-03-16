import React, { Component, ComponentType } from "react";
import classNames from "classnames";
import { action, computed } from "mobx";
import { observer } from "mobx-react";
import { Object3D } from "three";

import { SwitchesMenu, SwitchesMenuOption } from "../switches_menu/view";
import { Canvas, ObjectFit, Three } from "../three/three";

import { CameraController } from "./controller";
import { IconZoomIn, IconZoomOut, IconZoomReset } from "./icons";
import { CameraViewModel } from "./view_model";

export type Renderable = false | Object3D | undefined;

export interface CameraViewProps {
  switchMenuOptions?: SwitchesMenuOption[];
  viewModel: CameraViewModel;
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
export class CameraView extends Component<CameraViewProps> {
  static defaultProps = {
    viewType: "full",
    objectFit: "contain",
    allowPanAndZoom: false,
  } as const;

  render() {
    const { viewModel, viewType, allowPanAndZoom, onClick } = this.props;
    const controller = CameraController.of(viewModel);

    return (
      <div
        className={classNames("relative w-full h-full", {
          "bg-black": viewType === "full",
          "bg-transparent": viewType === "thumbnail",
        })}
      >
        {allowPanAndZoom ? (
          <Three
            stage={this.stage}
            objectFit={this.objectFit}
            onClick={onClick}
            onWheel={controller.zoomScroll}
            onMouseDown={controller.startPan}
            onMouseUp={controller.endPan}
            onMouseMove={controller.pan}
            renderScheduler={(callback) => callback()}
          />
        ) : (
          <Three
            stage={this.stage}
            objectFit={this.objectFit}
            onClick={onClick}
            renderScheduler={(callback) => callback()}
          />
        )}
        {viewType === "full" ? (
          <div className="text-white flex flex-col absolute top-0 right-0 bg-black/30 rounded-lg m-4">
            <SwitchesMenu dropdownMenuPosition="right" options={this.drawOptions} />
            {allowPanAndZoom ? (
              <div className="flex flex-col mb-2">
                <IconButton
                  className="hover:text-gray-300 py-2"
                  Icon={IconZoomIn}
                  title="Zoom In"
                  onClick={() => controller.zoomCenter(1)}
                />
                <IconButton
                  className="hover:text-gray-300 py-2"
                  Icon={IconZoomOut}
                  title="Zoom Out"
                  onClick={() => controller.zoomCenter(-1)}
                />
                <IconButton
                  className="hover:text-gray-300 py-2"
                  Icon={IconZoomReset}
                  title="Zoom Reset"
                  onClick={controller.resetCamera}
                />
              </div>
            ) : null}
          </div>
        ) : null}
        <div
          className={classNames("absolute bg-black/30 text-white text-sm leading-none", {
            "top-0 left-0 rounded-br px-2 pt-2 pb-1.5 text-sm": viewType === "thumbnail",
            "top-2 left-[50%] translate-x-[-50%] rounded p-2": viewType === "full",
          })}
        >
          {viewModel.name}
        </div>
      </div>
    );
  }

  @computed
  private get drawOptions(): SwitchesMenuOption[] {
    const { drawOptions } = this.props.viewModel;
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
      ...(this.props.switchMenuOptions ?? []),
    ];
  }

  @computed.struct
  private get objectFit(): ObjectFit {
    switch (this.props.objectFit) {
      case "fill":
        return { type: "fill" };
      case "contain":
      case "cover":
        return { type: this.props.objectFit, aspect: this.props.viewModel.imageAspectRatio };
    }
  }

  private readonly stage = (canvas: Canvas) => {
    // Pass an observable reference to the canvas size so the width/height in the view model updates
    // with the width/height of the Three canvas.
    this.props.viewModel.canvas = canvas;
    return computed(() => [this.props.viewModel.stage]);
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
