import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { LoadingIcon } from "../view";
import { themeDecorator } from "../../storybook_theme_decorator/view";

const meta: Meta<typeof LoadingIcon> = {
  title: "components/LoadingIcon",
  component: LoadingIcon,
  decorators: [themeDecorator],
};

export default meta;

type Story = StoryObj<typeof LoadingIcon>;

export const Animated: Story = {
  name: "default",
  render: () => {
    return <LoadingIcon />;
  },
};

export const CustomSize: Story = {
  name: "custom size",
  render: () => {
    return <LoadingIcon size={64} />;
  },
};
