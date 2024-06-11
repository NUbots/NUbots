import React from "react";
import { action } from "@storybook/addon-actions";
import type { Meta, StoryObj } from "@storybook/react";

import { lightAndDarkDecorator } from "../storybook/color_mode";

import { Button, ButtonProps } from "./button";

const meta: Meta<typeof Button> = {
  title: "components/Button",
  component: Button,
  decorators: [lightAndDarkDecorator({ className: "max-w-3xl" })],
  parameters: {
    controls: { expanded: true },
  },
  args: {
    className: "w-full",
  },
};

export default meta;

type Story = StoryObj<typeof Button>;

/** Action for when buttons are clicked */
const onClick = action("onClick");

/** A row with a label next to a some content */
function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex items-center justify-center gap-2 w-full">
      <span className="inline-block grow-0 shrink-0 w-48 text-right whitespace-nowrap tracking-wider uppercase text-sm mr-4 text-gray-500 dark:text-white/60">
        {label}
      </span>
      {children}
    </div>
  );
}

export const Variants: Story = {
  name: "Variants",
  args: {
    size: "normal",
  },
  parameters: {
    controls: { include: ["size"] },
  },
  render: ({ size }) => {
    /** A group of inline buttons with different icons and text */
    function ButtonSet(props: Partial<ButtonProps>) {
      return (
        <>
          <Button onClick={onClick} className="shrink-0" {...props}>
            Click Me!
          </Button>
          <Button
            onClick={onClick}
            className="shrink-0"
            iconBefore="account_circle"
            iconBeforeProps={{ fill: true }}
            {...props}
          >
            Icon Before
          </Button>
          <Button onClick={onClick} className="shrink-0" iconAfter="expand_more" {...props}>
            Icon After
          </Button>
        </>
      );
    }

    return (
      <div className="flex flex-col gap-1 items-center">
        <Row label="Transparent">
          <ButtonSet color="transparent" size={size} />
        </Row>
        <Row label="Transparent, disabled">
          <ButtonSet color="transparent" disabled size={size} />
        </Row>
        <Row label="Default">
          <ButtonSet color="default" size={size} />
        </Row>
        <Row label="Default, disabled">
          <ButtonSet color="default" disabled size={size} />
        </Row>
        <Row label="Primary">
          <ButtonSet color="primary" size={size} />
        </Row>
        <Row label="Primary, disabled">
          <ButtonSet color="primary" disabled size={size} />
        </Row>
      </div>
    );
  },
};

export const ContentAlignment: Story = {
  name: "Content Alignment",
  args: {
    size: "normal",
  },
  parameters: {
    controls: { include: ["size"] },
  },
  render: ({ size }) => {
    /** A group of buttons with different content alignments */
    return (
      <div className="flex gap-1 w-2x">
        <Button onClick={onClick} contentAlign="left" className="w-full" size={size}>
          Content Left
        </Button>
        <Button onClick={onClick} contentAlign="center" className="w-full" size={size}>
          Content Center
        </Button>
        <Button onClick={onClick} contentAlign="right" className="w-full" size={size}>
          Content Right
        </Button>
      </div>
    );
  },
};

export const Adjustable: Story = {
  name: "Adjustable",
  render: (props) => (
    <div className="flex items-center gap-2 pl-8">
      <Button {...props}>
        <span className="capitalize">{props.color ?? "Button"}</span>
      </Button>
    </div>
  ),
};
