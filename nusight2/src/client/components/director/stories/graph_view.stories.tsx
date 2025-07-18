import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { GraphView } from "../components/graph_view";
import { transformDirectorState } from "../model";

import raw from "./state.json";

const graph = transformDirectorState(raw as any);

const meta: Meta = {
  title: "components/director/GraphView",
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="w-screen h-screen p-4 overflow-auto">{story()}</div>],
};

export default meta;

type Story = StoryObj;

export const Default: Story = {
  render: () => <GraphView graph={graph} />,
};
