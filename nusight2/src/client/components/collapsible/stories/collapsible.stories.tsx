import React from "react";
import { action } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";
import { action as mobxAction, observable } from "mobx";
import { observer } from "mobx-react";

import { Select } from "../../select/view";
import { Collapsible } from "../view";

const meta: Meta<typeof Collapsible> = {
  title: "components/Collapsible",
  component: Collapsible,
  decorators: [
    (Story) => {
      return (
        <div className="max-w-[320px]">
          <Story />
        </div>
      );
    },
  ],
};

export default meta;

type Story = StoryObj<typeof Collapsible>;

const actions = {
  onToggle: action("onToggle"),
};

export const Default: Story = {
  name: "default",
  render: () => {
    return (
      <Collapsible open={true} onToggle={actions.onToggle}>
        <div>Collapsible content</div>
        <div>
          Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum, tenetur eveniet.
          Adipisci sed, labore eos molestias.
        </div>
      </Collapsible>
    );
  },
};

export const WithHeader: Story = {
  name: "with header",
  render: () => {
    const header = <div>Collapsible Header</div>;
    return (
      <Collapsible open={true} onToggle={actions.onToggle} header={header}>
        <div>Collapsible content</div>
        <div>
          Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum, tenetur eveniet.
          Adipisci sed, labore eos molestias.
        </div>
      </Collapsible>
    );
  },
};

export const Interactive: Story = {
  name: "interactive",
  render: () => {
    const model = observable({
      open: true,
      animate: true,
    });

    const onToggle = mobxAction(() => (model.open = !model.open));
    const onCheckboxChange = mobxAction((event: React.ChangeEvent<HTMLInputElement>) => {
      model.animate = event.target.checked;
    });

    const header = <div>Click to toggle</div>;
    const Component = observer(() => (
      <>
        <label className="flex gap-2 mb-3">
          <input type="checkbox" checked={model.animate} onChange={onCheckboxChange} />
          <span>Animate</span>
        </label>
        <Collapsible open={model.open} onToggle={onToggle} header={header} animate={model.animate}>
          <div>Collapsible content</div>
          <div>
            Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum, tenetur eveniet.
            Adipisci sed, labore eos molestias.
          </div>
        </Collapsible>
      </>
    ));

    return <Component />;
  },
};

export const WithOverflow: Story = {
  name: "with overflowing content",
  render: () => {
    const model = observable({
      open: true,
      animate: true,
    });

    const onToggle = mobxAction(() => (model.open = !model.open));
    const onCheckboxChange = mobxAction((event: React.ChangeEvent<HTMLInputElement>) => {
      model.animate = event.target.checked;
    });

    const header = <div>Click to toggle</div>;
    const Component = observer(() => (
      <>
        <label className="flex gap-2 mb-3">
          <input type="checkbox" checked={model.animate} onChange={onCheckboxChange} />
          <span>Animate</span>
        </label>
        <Collapsible open={model.open} onToggle={onToggle} header={header} animate={model.animate}>
          <div>Collapsible content</div>
          <div>
            Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum, tenetur eveniet.
            Adipisci sed, labore eos molestias.
          </div>
          <Select
            placeholder="Show overflowing options"
            options={[
              { id: 0, label: "Option 1" },
              { id: 1, label: "Option 2" },
              { id: 2, label: "Option 3" },
            ]}
            onChange={() => {}}
          />
        </Collapsible>
      </>
    ));

    return <Component />;
  },
};
