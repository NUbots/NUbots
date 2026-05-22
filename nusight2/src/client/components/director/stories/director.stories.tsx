import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { AppController } from "../../app/controller";
import { AppModel } from "../../app/model";
import { withRobotSelectorMenuBar } from "../../menu_bar/view";
import { DirectorController } from "../controller";
import { DirectorModel, transformDirectorState } from "../model";
import { DirectorView } from "../view";

import sampleState from "./state.json";

interface StoryProps {}

const meta: Meta<StoryProps> = {
  title: "components/director/DirectorView",
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="w-screen h-screen">{story()}</div>],
};

export default meta;

type Story = StoryObj<StoryProps>;

export const Default: Story = {
  name: "renders placeholder",
  render: () => {
    const appModel = AppModel.of();
    const model = DirectorModel.of(appModel);
    const controller = DirectorController.of();

    const firstRobot = model.robots[0];
    if (firstRobot) {
      model.graphsByRobot.set(firstRobot.id, transformDirectorState(sampleState as any));
      model.selectedRobot = firstRobot;
    }

    const appController = AppController.of();
    const Menu = withRobotSelectorMenuBar(appModel, appController.toggleRobotEnabled);

    return <DirectorView controller={controller} model={model} Menu={Menu} />;
  },
};
