import React from "react";
import { action } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";
import { action as mobxAction, observable } from "mobx";
import { observer } from "mobx-react";

import { SwitchesMenu, SwitchesMenuOption } from "../view";
import { ThemeDecorator } from "../../storybook_theme_decorator/view";

const meta: Meta<typeof SwitchesMenu> = {
  title: "components/SwitchesMenu",
  component: SwitchesMenu,
  decorators: [(story) => <div style={{ maxWidth: "350px" }}>{story()}</div>, ThemeDecorator],
};

export default meta;

type Story = StoryObj<typeof SwitchesMenu>;

const actions = {
  toggle: action("toggle"),
};

export const Empty: Story = {
  name: "empty",
  render: () => {
    return <SwitchesMenu options={[]} />;
  },
};

export const WithOptinos: Story = {
  name: "with options",
  render: () => {
    return <SwitchesMenu options={getOptions()} />;
  },
};

export const DropdownRight: Story = {
  name: "dropdown right",
  render: () => {
    const style = { display: "flex", justifyContent: "flex-end" };
    return (
      <div style={style}>
        <SwitchesMenu options={getOptions()} dropdownMenuPosition="right" />
      </div>
    );
  },
};

export const Interactive: Story = {
  name: "interactive",
  render: () => {
    const model = observable({
      options: getOptions().map(({ label, enabled }: SwitchesMenuOption, i) => {
        return {
          label,
          enabled,
          toggle: mobxAction(() => {
            model.options[i].enabled = !model.options[i].enabled;
          }),
        };
      }),
    });
    const Component = observer(() => <SwitchesMenu options={model.options} />);

    return <Component />;
  },
};

function getOptions(): SwitchesMenuOption[] {
  return [
    { label: "Lines", enabled: true, toggle: actions.toggle },
    { label: "Balls", enabled: true, toggle: actions.toggle },
    { label: "Goals", enabled: false, toggle: actions.toggle },
  ];
}
