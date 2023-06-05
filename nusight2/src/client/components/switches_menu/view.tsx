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
    <button className={style.button}>
      <IconMenu />
    </button>
  );
  return (
    <div className={style.switchesMenu}>
      <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownPosition={props.dropdownMenuPosition}>
        <div className={style.options}>
          {options.length === 0 && <div className={style.empty}>No options</div>}
          {options.map((option) => {
            return (
              <label key={option.label} className={style.option}>
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
