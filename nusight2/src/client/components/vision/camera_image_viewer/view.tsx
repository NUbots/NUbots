import React, { ComponentType } from "react";
import classNames from "classnames";
import { observer } from "mobx-react";

import { RobotModel } from "../../robot/model";
import { VisionCameraModel } from "../camera/model";
import { VisionCameraViewProps } from "../camera/view";
import { VisionRobotModel } from "../model";

import IconChevronLeft from "./icon_chevron_left";
import IconChevronRight from "./icon_chevron_right";
import IconClose from "./icon_close";
import style from "./style.module.css";

export const CameraImageViewer = observer(
  (props: {
    CameraView: ComponentType<VisionCameraViewProps>;
    selectedRobot: VisionRobotModel;
    selectedCamera: VisionCameraModel;
    onSelectRobot: (robot?: RobotModel) => void;
    onSelectCamera: (cameraIndex: number) => void;
    onPreviousCamera: () => void;
    onNextCamera: () => void;
  }) => {
    return (
      <div className="flex w-full h-full relative">
        <CameraImageViewerMain
          CameraView={props.CameraView}
          selectedRobot={props.selectedRobot}
          selectedCamera={props.selectedCamera}
          onSelectCamera={props.onSelectCamera}
          onPreviousCamera={props.onPreviousCamera}
          onNextCamera={props.onNextCamera}
        />
        <CameraImageViewerThumbnails
          CameraView={props.CameraView}
          selectedRobot={props.selectedRobot}
          onSelectCamera={props.onSelectCamera}
        />
      </div>
    );
  },
);

const CameraImageViewerMain = observer(
  (props: {
    CameraView: ComponentType<VisionCameraViewProps>;
    selectedRobot: VisionRobotModel;
    selectedCamera: VisionCameraModel;
    onSelectCamera: (cameraIndex: number) => void;
    onPreviousCamera: () => void;
    onNextCamera: () => void;
  }) => {
    return (
      <div className="flex-grow h-full relative">
        <props.CameraView
          key={props.selectedCamera.id}
          model={props.selectedCamera}
          robot={props.selectedRobot}
          viewType="full"
          objectFit="fill"
          allowPanAndZoom
        />
        <button
          className="absolute bg-transparent flex leading-none p-1.5 rounded-sm top-2 left-2 hover:bg-gray-100"
          onClick={() => props.onSelectCamera(-1)}
        >
          <IconClose className="text-nusight-500 h-8 w-8" />
        </button>
        <button
          className="absolute bg-transparent flex leading-none p-1.5 rounded-sm top-[50%] left-2 hover:bg-gray-100"
          onClick={props.onPreviousCamera}
        >
          <IconChevronLeft className="text-nusight-500 h-8 w-8" />
        </button>
        <button
          className="absolute bg-transparent flex leading-none p-1.5 rounded-sm top-[50%] right-2 hover:bg-gray-100"
          onClick={props.onNextCamera}
        >
          <IconChevronRight className="text-nusight-500 h-8 w-8" />
        </button>
      </div>
    );
  },
);

const CameraImageViewerThumbnails = observer(
  (props: {
    CameraView: ComponentType<VisionCameraViewProps>;
    selectedRobot: VisionRobotModel;
    onSelectCamera: (cameraIndex: number) => void;
  }) => {
    return (
      <div className="w-1/5 max-w-[216px] min-w-[120px] h-full bg-black border-l border-gray-700 relative">
        <div className="w-full h-full overflow-y-auto absolute top-0 right-0">
          <div className="grid grid-cols-1 auto-rows-[140px] gap-1 p-1.5">
            {props.selectedRobot?.cameraList.map((camera, index) => (
              <CameraImageViewerThumbnail
                key={camera.id}
                CameraView={props.CameraView}
                camera={camera}
                selectedRobot={props.selectedRobot}
                onClick={() => props.onSelectCamera(index)}
              />
            ))}
          </div>
        </div>
      </div>
    );
  },
);

const CameraImageViewerThumbnail = observer(
  (props: {
    CameraView: ComponentType<VisionCameraViewProps>;
    camera: VisionCameraModel;
    selectedRobot: VisionRobotModel;
    onClick: () => void;
  }) => {
    return (
      <div
        className={classNames(style.cameraThumbnail, "cursor-pointer relative", {
          [style.cameraThumbnailSelected]: props.camera.selected,
        })}
      >
        <props.CameraView
          model={props.camera}
          robot={props.selectedRobot}
          viewType="thumbnail"
          objectFit="contain"
          onClick={props.onClick}
          allowPanAndZoom={false}
        />
      </div>
    );
  },
);
