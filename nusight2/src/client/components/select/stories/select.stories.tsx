import React from "react";
import { action } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";
import { action as mobxAction, observable } from "mobx";
import { observer } from "mobx-react";

import { Icon } from "../../icon/view";
import { Option, Select } from "../view";
import { ThemeDecorator } from "../../storybook_theme_decorator/view";

const IconColor = () => <Icon>palette</Icon>;
const IconColoredCircle = (props: { color: string }) => (
  <Icon fill size="20" style={{ color: props.color }}>
    circle
  </Icon>
);

const meta: Meta<typeof Select> = {
  title: "components/Select",
  component: Select,
  decorators: [(story) => <div style={{ maxWidth: "350px" }}>{story()}</div>, ThemeDecorator],
};

export default meta;

type Story = StoryObj<typeof Select>;

const actions = {
  onChange: action("onChange"),
};

export const Default: Story = {
  name: "default",
  render: () => {
    return <Select options={[]} onChange={actions.onChange} placeholder="Select..." />;
  },
};

export const Empty: Story = {
  name: "empty",
  render: () => {
    const empty = (
      <div>
        <h4>No options</h4>
        <p>Add options to see them here</p>
      </div>
    );
    return <Select options={[]} onChange={actions.onChange} placeholder="Select..." empty={empty} />;
  },
};

export const WithOptions: Story = {
  name: "with options",
  render: () => {
    const options = getOptions();
    return <Select options={options} onChange={actions.onChange} placeholder="Select a color..." />;
  },
};

export const WithSelection: Story = {
  name: "with selection",
  render: () => {
    const options = getOptions();
    const selected = options[1];
    return (
      <Select options={options} selectedOption={selected} onChange={actions.onChange} placeholder="Select a color..." />
    );
  },
};

export const WithIcon: Story = {
  name: "with icon",
  render: () => {
    const options = getOptions();
    return (
      <Select options={options} onChange={actions.onChange} placeholder="Select a color..." icon={<IconColor />} />
    );
  },
};

export const WithOptionIcons: Story = {
  name: "with option icons",
  render: () => {
    const options = getOptionsWithIcons();
    return (
      <Select options={options} onChange={actions.onChange} placeholder="Select a color..." icon={<IconColor />} />
    );
  },
};

export const Interactive: Story = {
  name: "interactive",
  render: () => {
    const options = getOptions();
    const model = observable({
      options,
      selectedOption: options[1],
    });
    const onChange = mobxAction((option: Option) => (model.selectedOption = option));
    const Component = observer(() => (
      <Select
        options={model.options}
        selectedOption={model.selectedOption}
        onChange={onChange}
        placeholder="Select a color..."
        icon={<IconColor />}
      />
    ));

    return <Component />;
  },
};

function getOptions(): Option[] {
  return [
    { id: "red", label: "Red" },
    { id: "green", label: "Green" },
    { id: "blue", label: "Blue" },
  ];
}

function getOptionsWithIcons(): Option[] {
  return [
    { id: "red", label: "Red", icon: <IconColoredCircle color="red" /> },
    { id: "green", label: "Green", icon: <IconColoredCircle color="green" /> },
    { id: "blue", label: "Blue", icon: <IconColoredCircle color="blue" /> },
    { id: "no color", label: "No Color" },
  ];
}
