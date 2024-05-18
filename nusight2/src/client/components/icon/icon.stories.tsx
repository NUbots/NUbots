import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Icon } from "./view";
import { themeDecorator } from "../storybook_theme_decorator/view";

const meta: Meta<typeof Icon> = {
  title: "components/Icon",
  component: Icon,
  argTypes: {
    fill: { control: "boolean" },
    weight: { control: "inline-radio" },
    grade: { control: "inline-radio" },
    opticalSize: { control: "inline-radio" },
    size: { control: "inline-radio" },
    flip: { control: "inline-radio" },
    rotate: { control: "inline-radio" },
  },
  decorators: [themeDecorator]
};

export default meta;

type Story = StoryObj<typeof Icon>;

export const Default: Story = {
  name: "default",
  render: () => {
    return (
      <div className="flex gap-4">
        <Icon>check_circle</Icon>
        <Icon>home</Icon>
        <Icon>delete</Icon>
        <Icon>arrow_back</Icon>
        <Icon>arrow_forward</Icon>
        <Icon>refresh</Icon>
      </div>
    );
  },
};

export const AdjustableIcons: Story = {
  name: "adjustable icons",
  render: (props) => {
    const { fill, weight, grade, opticalSize, flip, rotate, size } = props;

    return (
      <>
        <div className="flex gap-4">
          <Icon
            fill={fill}
            weight={weight}
            grade={grade}
            opticalSize={opticalSize}
            flip={flip}
            rotate={rotate}
            size={size}
          >
            check_circle
          </Icon>
          <Icon
            fill={fill}
            weight={weight}
            grade={grade}
            opticalSize={opticalSize}
            flip={flip}
            rotate={rotate}
            size={size}
          >
            home
          </Icon>
          <Icon
            fill={fill}
            weight={weight}
            grade={grade}
            opticalSize={opticalSize}
            flip={flip}
            rotate={rotate}
            size={size}
          >
            delete
          </Icon>
          <Icon
            fill={fill}
            weight={weight}
            grade={grade}
            opticalSize={opticalSize}
            flip={flip}
            rotate={rotate}
            size={size}
          >
            arrow_back
          </Icon>
          <Icon
            fill={fill}
            weight={weight}
            grade={grade}
            opticalSize={opticalSize}
            flip={flip}
            rotate={rotate}
            size={size}
          >
            arrow_forward
          </Icon>
          <Icon
            fill={fill}
            weight={weight}
            grade={grade}
            opticalSize={opticalSize}
            flip={flip}
            rotate={rotate}
            size={size}
          >
            refresh
          </Icon>
        </div>
        <p className="mt-4">Open the Storybook Controls to adjust the icons</p>
      </>
    );
  },
};
