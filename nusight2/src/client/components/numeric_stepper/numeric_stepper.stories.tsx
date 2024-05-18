import React, { useState } from "react";
import { Meta, StoryObj } from "@storybook/react";

import { NumericStepper, NumericStepperProps } from "./view";
import { themeDecorator } from "../storybook_theme_decorator/view";

const meta: Meta<typeof NumericStepper> = {
  title: "components/NumericStepper",
  component: NumericStepper,
  decorators: [themeDecorator],
};

export default meta;

type Story = StoryObj<typeof NumericStepper>;

function LightAndDarkStepper(props: NumericStepperProps) {
  return (
    <div className="inline-flex flex-col">
      <div className="border border-b-0 p-5 rounded-t-lg">
        <NumericStepper {...props} />
      </div>
      <div className="dark bg-slate-800 p-5 rounded-b-lg">
        <NumericStepper {...props} />
      </div>
    </div>
  );
}

export const Default: Story = {
  render: () => {
    const [value, setValue] = useState(50);
    return <LightAndDarkStepper value={value} onChange={setValue} />;
  },
};

export const FormatValue: Story = {
  render: () => {
    const [value, setValue] = useState(50);
    return <LightAndDarkStepper value={value} onChange={setValue} formatValue={(value) => value + "%"} />;
  },
};

export const Resettable: Story = {
  render: () => {
    const [value, setValue] = useState(50);
    return (
      <LightAndDarkStepper
        value={value}
        onChange={setValue}
        formatValue={(value) => value + "%"}
        canReset
        defaultValue={50}
      />
    );
  },
};

export const WithLabel: Story = {
  render: () => {
    const [value, setValue] = useState(50);
    return (
      <LightAndDarkStepper
        label="Zoom"
        value={value}
        onChange={setValue}
        formatValue={(value) => value + "%"}
        canReset
        defaultValue={50}
      />
    );
  },
};

export const MinMaxStep: Story = {
  render: () => {
    const [value, setValue] = useState(0.5);
    return (
      <>
        <p className="mb-3">Min: 0, Max: 1, Step: 0.1</p>
        <LightAndDarkStepper
          label="Zoom"
          value={value}
          onChange={setValue}
          formatValue={(value) => value.toFixed(1) + "𝑥"}
          min={0}
          max={1}
          step={0.1}
          canReset={true}
          defaultValue={0.5}
        />
      </>
    );
  },
};

export const Adjustable: Story = {
  render: (props) => {
    const [value, setValue] = useState(props.value ?? 50);
    return (
      <>
        <p className="mb-3">Open the Storybook controls to adjust the stepper.</p>
        <LightAndDarkStepper
          label={props.label}
          value={value}
          onChange={setValue}
          canReset={props.canReset}
          defaultValue={props.canReset ? props.defaultValue ?? 50 : 50}
          min={props.min}
          max={props.max}
          step={props.step}
          increaseButtonTitle={props.increaseButtonTitle}
          decreaseButtonTitle={props.decreaseButtonTitle}
          resetButtonTitle={props.resetButtonTitle}
          valueWidth={props.valueWidth}
        />
      </>
    );
  },
};
