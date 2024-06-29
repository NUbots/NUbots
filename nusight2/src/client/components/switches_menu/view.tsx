import React from "react";
import { observer } from "mobx-react";

import { dropdownContainer } from "../dropdown_container/view";
import { Switch } from "../switch/view";

import IconMenu from "./icon_menu";
import style from "./style.module.css";

export type SwitchesMenuOption = {
  label: string;
  enabled: boolean;
  toggle(): void;
};

export type SwitchesMenuProps = {
  dropdownMenuPosition?: "left" | "right";
  options: SwitchesMenuOption[];
};

export const SwitchesMenu = observer((props: SwitchesMenuProps) => {
  const { options } = props;
  const dropdownToggle = (
    <button
      className={
        "flex flex-col items-center justify-center h-14 px-4 bg-transparent cursor-pointer  transition-colors duration-200 outline-none fill-gray-100 hover:fill-gray-450"
      }
    >
      <IconMenu />
    </button>
  );
  return (
    <div>
      <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownPosition={props.dropdownMenuPosition}>
        <div className="bg-auto-surface-1 shadow-md rounded-md text-auto-primary">
          {options.length === 0 && <div className="bg-300 dark:bg-gray-700 p-4 rounded-md">No options</div>}
          {options.map((option) => {
            return (
              <label key={option.label} className="items-center flex justify-between p-4 hover:bg-auto-contrast-1 ">
                <span className={style.optionLabel}>{option.label}</span>
                <Switch on={option.enabled} onChange={option.toggle} />
              </label>
            );
          })}
        </div>
      </EnhancedDropdown>
    </div>
  );
});

const EnhancedDropdown = dropdownContainer();
