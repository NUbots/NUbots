import React from "react";
import type { Meta, StoryObj } from "@storybook/react";

import { IconButton } from "../icon_button/view";
import { NumericStepper } from "../numeric_stepper/view";
import { Select } from "../select/view";
import { LightAndDark } from "../storybook/color_mode";

import { Button, ButtonProps } from "./button";

type StoryProps = Pick<ButtonProps, "size" | "color" | "disabled">;

const meta: Meta<StoryProps> = {
  title: "components/Button",
  decorators: [],
  parameters: {
    controls: { expanded: true },
  },
};

export default meta;

type Story = StoryObj<StoryProps>;

export const ButtonWithRelated: Story = {
  render: () => {
    function Row({ label, ...props }: StoryProps & { label: string }) {
      return (
        <tr>
          <td className="whitespace-nowrap uppercase text-right text-sm mr-4 text-gray-500 dark:text-white/60">
            {label}
          </td>
          <td>
            <Button {...props}>Button</Button>
          </td>
          <td>
            <div className="flex gap-1">
              <IconButton {...props}>favorite</IconButton>
              <IconButton {...props}>account_circle</IconButton>
              <IconButton {...props}>check_box</IconButton>
            </div>
          </td>
          <td>
            <NumericStepper defaultValue={0} value={0} canReset={true} onChange={() => {}} />
          </td>
          <td>
            <Select options={[]} placeholder="Select" onChange={() => {}} />
          </td>
        </tr>
      );
    }

    return (
      <>
        <p className="text-center mb-3 max-w-3xl">Example of how Button appears side-by-side with related components</p>
        <LightAndDark opts={{ className: "max-w-3xl" }}>
          {() => (
            <table className="border-separate border-spacing-2">
              <thead>
                <tr className="text-center whitespace-nowrap uppercase tracking-wide text-sm text-gray-500 dark:text-white/60">
                  <td></td>
                  <td>Button</td>
                  <td>IconButton</td>
                  <td>NumericStepper</td>
                  <td>Select</td>
                </tr>
              </thead>
              <tbody>
                <Row label="Transparent" color="transparent" />
                <Row label="Transparent, disabled" color="transparent" disabled />
                <Row label="Default" />
                <Row label="Default, disabled" disabled />
                <Row label="Primary" color="primary" />
                <Row label="Primary, disabled" color="primary" />
              </tbody>
            </table>
          )}
        </LightAndDark>
      </>
    );
  },
};
