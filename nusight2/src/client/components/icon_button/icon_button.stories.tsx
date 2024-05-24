import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Icon, IconProps } from "../icon/view";
import { ThemeDecorator } from "../storybook_theme_decorator/view";

import { IconButton, IconButtonProps } from "./view";

const meta: Meta<typeof IconButton> = {
  title: "components/IconButton",
  component: IconButton,
  decorators: [ThemeDecorator],
};

export default meta;

type Story = StoryObj<typeof IconButton>;

function ButtonCard({ children }: { children: React.ReactNode }) {
  return (
    <div className="relative border-2 border-gray-300 dark:border-gray-700 p-5 rounded-lg overflow-hidden max-w-2xl">
      <span className="absolute top-0 left-0 bg-transparent py-1.5 px-2.5 rounded-br border-r-2 border-b-2 border-gray-300 dark:border-gray-700 flex">
        <Icon size="20">light_mode</Icon>
      </span>
      {children}
    </div>
  );
}

function ButtonRow({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex items-center justify-center gap-2">
      <span className="inline-block w-[180px] text-right whitespace-nowrap tracking-wider uppercase text-sm mr-4 text-gray-500 dark:text-white/60">
        {label}
      </span>
      {children}
    </div>
  );
}

function IconButtons(props: Omit<IconButtonProps, "children">) {
  return (
    <div className="flex items-center gap-2">
      <IconButton {...props}>home</IconButton>
      <IconButton {...props}>delete</IconButton>
      <IconButton {...props}>arrow_back</IconButton>
      <IconButton {...props}>arrow_forward</IconButton>
      <IconButton {...props}>repeat</IconButton>
      <IconButton {...props}>repeat_on</IconButton>
      <IconButton {...props}>
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <path d="M20.24 12.24a6 6 0 0 0-8.49-8.49L5 10.5V19h8.5z"></path>
          <line x1="16" y1="8" x2="2" y2="22"></line>
          <line x1="17.5" y1="15" x2="9" y2="15"></line>
        </svg>
      </IconButton>
      <IconButton {...props}>
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <polygon points="12 2 22 8.5 22 15.5 12 22 2 15.5 2 8.5 12 2"></polygon>
          <line x1="12" y1="22" x2="12" y2="15.5"></line>
          <polyline points="22 8.5 12 15.5 2 8.5"></polyline>
          <polyline points="2 15.5 12 8.5 22 15.5"></polyline>
          <line x1="12" y1="2" x2="12" y2="8.5"></line>
        </svg>
      </IconButton>
    </div>
  );
}

export const AllVariants: Story = {
  render: () => {
    return (
      <>
        <ButtonCard>
          <div className="flex flex-col gap-3">
            <ButtonRow label="Transparent">
              <IconButtons color="transparent" />
            </ButtonRow>
            <ButtonRow label="Transparent, Disabled">
              <IconButtons color="transparent" disabled />
            </ButtonRow>
            <ButtonRow label="Default">
              <IconButtons color="default" />
            </ButtonRow>
            <ButtonRow label="Default, Disabled">
              <IconButtons color="default" disabled />
            </ButtonRow>
            <ButtonRow label="Primary">
              <IconButtons color="primary" />
            </ButtonRow>
            <ButtonRow label="Primary, Disabled">
              <IconButtons color="primary" disabled />
            </ButtonRow>
          </div>
        </ButtonCard>
      </>
    );
  },
};

export const AllVariantsLarge: Story = {
  name: "All Variants, Large",

  render: () => {
    return (
      <>
        <ButtonCard>
          <div className="flex flex-col gap-3">
            <ButtonRow label="Transparent">
              <IconButtons size="large" color="transparent" />
            </ButtonRow>
            <ButtonRow label="Transparent, Disabled">
              <IconButtons size="large" color="transparent" disabled />
            </ButtonRow>
            <ButtonRow label="Default">
              <IconButtons size="large" color="default" />
            </ButtonRow>
            <ButtonRow label="Default, Disabled">
              <IconButtons size="large" color="default" disabled />
            </ButtonRow>
            <ButtonRow label="Primary">
              <IconButtons size="large" color="primary" />
            </ButtonRow>
            <ButtonRow label="Primary, Disabled">
              <IconButtons size="large" color="primary" disabled />
            </ButtonRow>
          </div>
        </ButtonCard>
      </>
    );
  },
};

interface AdjustableStoryProps {
  color: IconButtonProps["color"];
  size: IconButtonProps["size"];
  iconFill: IconProps["fill"];
  iconWeight: IconProps["weight"];
  iconFlip: IconProps["flip"];
  iconRotate: IconProps["rotate"];
  disabled: IconButtonProps["disabled"];
}

export const Adjustable: StoryObj<React.FunctionComponent<AdjustableStoryProps>> = {
  args: {
    color: "default",
    size: "normal",
    iconFill: false,
    iconWeight: "300",
    iconFlip: "none",
    iconRotate: "0",
    disabled: false,
  },

  argTypes: {
    color: { control: "inline-radio", options: ["default", "primary", "transparent"] },
    size: { control: "inline-radio", options: ["normal", "large"] },
    iconFill: { control: "boolean" },
    iconWeight: { control: "inline-radio", options: ["100", "200", "300", "400", "500", "600", "700"] },
    iconFlip: { control: "inline-radio", options: ["none", "horizontal", "vertical", "both"] },
    iconRotate: { control: "inline-radio", options: ["0", "90", "180", "270"] },
    disabled: { control: "boolean" },
  },

  render: (props) => {
    const { iconFill, iconWeight, iconFlip, iconRotate, ...mainProps } = props;
    const iconProps = { fill: iconFill, weight: iconWeight, flip: iconFlip, rotate: iconRotate };

    return (
      <>
        <p className="text-center mb-3 max-w-2xl">Open the Storybook controls to adjust the buttons.</p>
        <ButtonCard>
          <div className="flex justify-center">
            <IconButtons {...mainProps} iconProps={iconProps} />
          </div>
        </ButtonCard>
      </>
    );
  },
};
