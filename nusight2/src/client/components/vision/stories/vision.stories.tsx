import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Projection } from "../../../../shared/math/projection";
import { AppController } from "../../app/controller";
import { AppModel } from "../../app/model";
import { CameraParams } from "../../camera/camera_params";
import { ImageFormat } from "../../camera/image";
import { Lens } from "../../camera/lens";
import { withRobotSelectorMenuBar } from "../../menu_bar/view";
import { RobotModel } from "../../robot/model";
import { VisionController } from "../controller";
import { VisionModel } from "../model";
import { VisionView } from "../view";
import { VisionCameraModel } from "../vision_camera/model";
import { VisionCameraViewProps } from "../vision_camera/view";

interface StoryProps {}

const meta: Meta<StoryProps> = {
  title: "components/vision/Layout",
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="w-screen h-screen">{story()}</div>],
};

export default meta;

export const Default: StoryObj<StoryProps> = {
  name: "renders",
  render: () => {
    const appModel = AppModel.of({
      robots: [
        RobotModel.of({
          id: "1",
          name: "Robot #1",
          enabled: true,
          connected: true,
          address: "127.0.0.1",
          port: 1,
          type: "nuclearnet-peer",
        }),
        RobotModel.of({
          id: "2",
          name: "Robot #2",
          enabled: true,
          connected: true,
          address: "127.0.0.2",
          port: 2,
          type: "nuclearnet-peer",
        }),
        RobotModel.of({
          id: "3",
          name: "Robot #3",
          enabled: true,
          connected: true,
          address: "127.0.0.3",
          port: 3,
          type: "nuclearnet-peer",
        }),
        RobotModel.of({
          id: "4",
          name: "Robot #4",
          enabled: true,
          connected: true,
          address: "127.0.0.4",
          port: 4,
          type: "nuclearnet-peer",
        }),
      ],
    });
    const model = VisionModel.of(appModel);
    model.visionRobots.forEach((robot) => {
      robot.cameras.set(
        1,
        VisionCameraModel.of({
          id: 1,
          name: "Camera #1",
          image: {
            type: "element",
            width: 320,
            height: 240,
            element: {} as HTMLImageElement,
            format: ImageFormat.JPEG,
          },
          params: new CameraParams({
            Hcw: Matrix4.of(),
            lens: new Lens({ projection: Projection.RECTILINEAR, focalLength: 1 }),
          }),
        }),
      );
      robot.cameras.set(
        2,
        VisionCameraModel.of({
          id: 2,
          name: "Camera #2",
          image: {
            type: "element",
            width: 320,
            height: 240,
            element: {} as HTMLImageElement,
            format: ImageFormat.JPEG,
          },
          params: new CameraParams({
            Hcw: Matrix4.of(),
            lens: new Lens({ projection: Projection.RECTILINEAR, focalLength: 1 }),
          }),
        }),
      );
    });
    const appController = AppController.of();
    const Menu = withRobotSelectorMenuBar(appModel, appController.toggleRobotEnabled);
    const CameraView = (props: VisionCameraViewProps) => (
      <div
        style={{
          position: "absolute",
          width: "100%",
          height: "100%",
          color: "white",
          backgroundColor: "black",
          border: "1px dashed white",
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          fontSize: "200%",
          boxSizing: "border-box",
        }}
      >
        {props.model.name}
      </div>
    );
    const controller = VisionController.of();
    return <VisionView controller={controller} model={model} Menu={Menu} CameraView={CameraView} />;
  },
};
