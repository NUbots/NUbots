import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { ProviderView } from "../components/provider_view";
import { transformDirectorState } from "../model";

import raw from "./state.json";

const graph = transformDirectorState(raw as any);
const providerExample = graph.providersById["450"];

const meta: Meta<typeof ProviderView> = {
  title: "components/director/ProviderView",
  component: ProviderView,
  parameters: {
    layout: "centered",
  },
};

export default meta;

type Story = StoryObj<typeof ProviderView>;

export const Default: Story = {
  render: () => <ProviderView provider={providerExample} />,
};
