import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import raw from "./state.json";
import { transformDirectorState } from "../model";
import { TaskChip } from "../components/task_chip";

const graph = transformDirectorState(raw as any);
const taskExample = graph.groupsById["5715460735790508797"].subtasks[0];

const meta: Meta<typeof TaskChip> = {
  title: "components/director/TaskChip",
  component: TaskChip,
};

export default meta;

type Story = StoryObj<typeof TaskChip>;

export const Default: Story = {
  render: () => <TaskChip task={taskExample} />,
};
