import React from "react";
import { action } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";

import { Button } from "../../button/button";
import { Icon } from "../../icon/view";
import { lightOrDarkDecorator } from "../../storybook/color_mode";
import { ContextMenuProvider, useContextMenu } from "../context_menu";

const meta: Meta = {
  title: "components/ContextMenu",
  decorators: [lightOrDarkDecorator({ className: "max-w-xl" })],
};

export default meta;

export const Default: StoryObj = {
  render: () => {
    const Story = () => {
      const openContextMenu = useContextMenu(
        () => [
          { title: "Option 1", action: action("Option 1 pressed") },
          { type: "divider" },
          { title: "Option with Icon", icon: "person", action: action("Option with icon pressed") },
          { title: "Option with Hint", hint: "Hint Text", action: action("Option with hint pressed") },
          { title: "Disabled Option", disabled: true, action: action("Disabled option") },
          { type: "divider" },
          {
            type: "submenu",
            title: "Extra options",
            items: [
              { title: "Submenu Option 1", action: action("Submenu Option 1 pressed") },
              { title: "Submenu Option 2", action: action("Submenu Option 2 pressed") },
              {
                type: "submenu",
                title: "Sub-submenu",
                items: [
                  { title: "Option 1", action: action("Sub-submenu option 1 pressed") },
                  { title: "Option 2", action: action("Sub-submenu option 2 pressed") },
                ],
              },
            ],
          },
          {
            title: "Option with Custom Icon",
            icon: (
              <Icon size="20" className="text-nusight-500">
                check
              </Icon>
            ),
            action: action("Option with custom icon pressed"),
          },
        ],
        [],
      );

      const openContextMenu2 = useContextMenu(
        () => [
          { title: "Option 1", action: action("Option 1 pressed") },
          { type: "divider" },
          { title: "Option with Icon", icon: "person", action: action("Option with icon pressed") },
        ],
        [],
      );
      return (
        <div className="flex gap-2">
          <Button onContextMenu={openContextMenu}>Right Click Me!</Button>
          <Button onContextMenu={openContextMenu2}>Right Click Me!</Button>
        </div>
      );
    };

    return (
      <div className="flex justify-center">
        <ContextMenuProvider>
          <Story />
        </ContextMenuProvider>
      </div>
    );
  },
};
