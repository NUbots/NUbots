import React from "react";
import type { Meta, StoryObj } from "@storybook/react";

import { Checkbox } from "./checkbox";

const meta: Meta<typeof Checkbox> = {
  title: "components/Checkbox",
  component: Checkbox,
};

export default meta;

type Story = StoryObj<typeof Checkbox>;

export const Default: Story = {
  name: "default",
  render: (props) => {
    return (
      <>
        <p className="mb-4 text-sm">Use the Storybook controls to change the checkbox.</p>
        <Checkbox {...props} />
      </>
    );
  },
};
