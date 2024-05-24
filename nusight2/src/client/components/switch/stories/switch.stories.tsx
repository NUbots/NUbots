import React from "react";
import { action } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";
import { action as mobxAction, observable } from "mobx";
import { observer } from "mobx-react";

import { ThemeDecorator } from "../../storybook_theme_decorator/view";
import { Switch } from "../view";

const meta: Meta<typeof Switch> = {
  title: "components/Switch",
  component: Switch,
  decorators: [ThemeDecorator],
};

export default meta;

type Story = StoryObj<typeof Switch>;

export const on: Story = {
  name: "on",
  render: () => {
    return <Switch on={true} onChange={action("onChange")} />;
  },
};

export const off: Story = {
  name: "off",
  render: () => {
    return <Switch on={false} onChange={action("onChange")} />;
  },
};

export const Interactive: Story = {
  name: "Interactive",
  render: () => {
    const model = observable({ on: false });
    const onChange = mobxAction(() => (model.on = !model.on));
    const Component = observer(() => <Switch on={model.on} onChange={onChange} />);
    return <Component />;
  },
};
