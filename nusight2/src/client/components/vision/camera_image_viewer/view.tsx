import React, { ComponentType, useMemo } from "react";
import classNames from "classnames";
import { observer } from "mobx-react";

import { IconButton } from "../../icon_button/view";
import { ResizeContainer } from "../../resize_container/resize_container";
import { ResizePanel } from "../../resize_container/resize_panel";
import { RobotModel } from "../../robot/model";
import { VisionRobotModel } from "../model";
import { VisionCameraModel } from "../vision_camera/model";
import { VisionCameraViewProps } from "../vision_camera/view";
import { VisionCameraViewModel } from "../vision_camera/view_model";

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
    children?: React.ReactNode;
  }) => {
    return (
      <ResizeContainer horizontal className="w-full h-full flex dark">
        <ResizePanel minSize={250}>
          <CameraImageViewerMain
            CameraView={props.CameraView}
            selectedRobot={props.selectedRobot}
            selectedCamera={props.selectedCamera}
            onSelectCamera={props.onSelectCamera}
            onPreviousCamera={props.onPreviousCamera}
            onNextCamera={props.onNextCamera}
          >
            {props.children}
          </CameraImageViewerMain>
        </ResizePanel>
        <ResizePanel initialRatio={0} minSize={150} maxSize={550}>
          <CameraImageViewerThumbnails
            CameraView={props.CameraView}
            selectedRobot={props.selectedRobot}
            onSelectCamera={props.onSelectCamera}
          />
        </ResizePanel>
      </ResizeContainer>
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
    children?: React.ReactNode;
  }) => {
    const cameraViewModel = useMemo(
      () => VisionCameraViewModel.of(props.selectedCamera, props.selectedRobot),
      [props.selectedCamera, props.selectedRobot],
    );
    return (
      <div className="flex-grow h-full relative dark">
        <props.CameraView
          key={props.selectedCamera.id}
          model={props.selectedCamera}
          viewModel={cameraViewModel}
          robot={props.selectedRobot}
          viewType="full"
          objectFit="fill"
          allowPanAndZoom
        />
        <IconButton
          className="absolute top-2 left-2"
          color="transparent"
          size="large"
          iconProps={{ weight: 400 }}
          onClick={() => props.onSelectCamera(-1)}
        >
          close
        </IconButton>
        <IconButton
          className="absolute top-[50%] left-2"
          color="transparent"
          size="large"
          iconProps={{ weight: 400 }}
          onClick={props.onPreviousCamera}
        >
          chevron_left
        </IconButton>
        <IconButton
          className="absolute top-[50%] right-2"
          color="transparent"
          size="large"
          iconProps={{ weight: 400 }}
          onClick={props.onNextCamera}
        >
          chevron_right
        </IconButton>
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
      <div className="h-full bg-black border-l border-gray-700 relative">
        <div className="w-full h-full overflow-y-auto absolute top-0 right-0">
          <div className="grid grid-cols-1 gap-1 p-1.5">
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
    const cameraViewModel = useMemo(
      () => VisionCameraViewModel.of(props.camera, props.selectedRobot),
      [props.camera, props.selectedRobot],
    );
    return (
      <div
        className={classNames(style.cameraThumbnail, "cursor-pointer relative aspect-video border border-gray-900", {
          [style.cameraThumbnailSelected]: props.camera.selected,
        })}
      >
        <props.CameraView
          model={props.camera}
          viewModel={cameraViewModel}
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
