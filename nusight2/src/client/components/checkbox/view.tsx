import React from "react";
import { ChangeEvent } from "react";
import classNames from "classnames";

import IconCheck from "./icon_check";
import style from "./style.module.css";

export interface CheckboxProps {
  checked: boolean;
  disabled?: boolean;

  onChange(event: ChangeEvent<HTMLInputElement>): void;
}

export const Checkbox = (props: CheckboxProps) => {
  const { checked, disabled, onChange } = props;

  const backgroundClassName = classNames(style.background, {
    [style.checked]: checked,
    [style.disabled]: disabled,
  });

  const iconClassName = classNames(style.checkIcon, {
    [style.checkIconChecked]: checked,
  });

  return (
    <span className={style.checkbox}>
      <input
        type="checkbox"
        className={style.nativeControl}
        checked={checked}
        disabled={disabled}
        onChange={onChange}
      />
      <span className={backgroundClassName}>
        <IconCheck className={iconClassName} />
      </span>
    </span>
  );
};
