import React from "react";
import { action } from "@storybook/addon-actions";
import type { Meta, StoryObj } from "@storybook/react";

import { Icon } from "../icon/view";

import { Button } from "./view";

const meta: Meta<typeof Button> = {
  title: "components/Button",
  component: Button,
};

export default meta;

type Story = StoryObj<typeof Button>;

const onClick = action("onClick");

const IconBefore = (
  <Icon size={20} fill>
    account_circle
  </Icon>
);
const IconAfter = <Icon size={20}>expand_more</Icon>;

export const Default: Story = {
  name: "default",
  render: (props) => {
    return (
      <Button {...props}>
        <span className="capitalize">{props.type ?? "Button"}</span>
      </Button>
    );
  },
};

export const Types: Story = {
  name: "types",
  render: () => {
    return (
      <div className="flex gap-1">
        <Button onClick={onClick}>Normal</Button>
        <Button type="primary" onClick={onClick}>
          Primary
        </Button>
      </div>
    );
  },
};

export const Fullwidth: Story = {
  name: "fullwidth",
  render: () => {
    return (
      <div className="flex flex-col gap-1">
        <Button fullwidth onClick={onClick}>
          Normal
        </Button>
        <Button type="primary" fullwidth onClick={onClick}>
          Primary
        </Button>
      </div>
    );
  },
};

export const AlignedLeft: Story = {
  name: "aligned left",
  render: () => {
    return (
      <div className="flex flex-col gap-1">
        <Button fullwidth textAlign="left" onClick={onClick}>
          Normal
        </Button>
        <Button type="primary" textAlign="left" fullwidth onClick={onClick}>
          Primary
        </Button>
      </div>
    );
  },
};

export const AlignedRight: Story = {
  name: "aligned right",
  render: () => {
    return (
      <div className="flex flex-col gap-1">
        <Button fullwidth textAlign="right" onClick={onClick}>
          Normal
        </Button>
        <Button type="primary" textAlign="right" fullwidth onClick={onClick}>
          Primary
        </Button>
      </div>
    );
  },
};

export const IconBeforeStory: Story = {
  name: "icon before",
  render: () => {
    return (
      <div className="flex flex-col gap-1">
        <Button iconBefore={IconBefore} onClick={onClick}>
          Button
        </Button>
        <Button iconBefore={IconBefore} fullwidth onClick={onClick}>
          Fullwidth
        </Button>
        <Button iconBefore={IconBefore} fullwidth textAlign="left" onClick={onClick}>
          Fullwidth, aligned left
        </Button>
        <Button iconBefore={IconBefore} fullwidth textAlign="right" onClick={onClick}>
          Fullwidth, aligned right
        </Button>
      </div>
    );
  },
};

export const IconAfterStory: Story = {
  name: "icon after",
  render: () => {
    return (
      <div className="flex flex-col gap-1">
        <Button iconAfter={IconAfter} onClick={onClick}>
          Button
        </Button>
        <Button iconAfter={IconAfter} fullwidth onClick={onClick}>
          Button, fullwidth
        </Button>
        <Button iconAfter={IconAfter} fullwidth textAlign="left" onClick={onClick}>
          Fullwidth, aligned left
        </Button>
        <Button iconAfter={IconAfter} iconAfterAlignedRight fullwidth textAlign="left" onClick={onClick}>
          Fullwidth, aligned left, icon aligned right
        </Button>
        <Button iconAfter={IconAfter} fullwidth textAlign="right" onClick={onClick}>
          Fullwidth, aligned right
        </Button>
      </div>
    );
  },
};

export const Disabled: Story = {
  name: "disabled",
  render: () => {
    return (
      <div className="flex gap-1">
        <Button disabled>Normal</Button>
        <Button type="primary" disabled>
          Primary
        </Button>
      </div>
    );
  },
};
