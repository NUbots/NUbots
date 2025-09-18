import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { ProviderGroupView } from "../components/provider_group_view";
import { transformDirectorState } from "../model";

import raw from "./state.json";

const graph = transformDirectorState(raw as any);
const groupExample = graph.groupsById["5715460735790508797"];

const meta: Meta<typeof ProviderGroupView> = {
  title: "components/director/ProviderGroupView",
  component: ProviderGroupView,
  parameters: {
    layout: "centered",
  },
};

export default meta;

type Story = StoryObj<typeof ProviderGroupView>;

export const Default: Story = {
  render: () => <ProviderGroupView group={groupExample} />,
};
