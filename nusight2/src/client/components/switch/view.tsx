import React from "react";
import { ChangeEvent } from "react";
import classNames from "classnames";

import style from "./style.module.css";

export interface SwitchProps {
  on: boolean;
  disabled?: boolean;

  onChange(event: ChangeEvent<HTMLInputElement>): void;
}

// TODO: Fix vertical alignment of switch

export const Switch = (props: SwitchProps) => {
  const { disabled, on } = props;
  const trackClassName = classNames("rounded-full h-4 w-full relative", {
    ["bg-gray-400"]: !on,
    ["bg-blue-400 opacity-50"]: on,
  });
  const thumbClassName = classNames("rounded-full shadow-md h-6 left-0 absolute w-6 top-0 transition-transform duration-250 ease-in-out", {
    ["bg-gray-100"]: !on,
    ["bg-blue-400 translate-x-4"]: on,
  });
  return (
    <span className={"flex items-center cursor-pointer h-5 relative w-10"}>
      <span className={trackClassName} />
      <span role="thumb" className={thumbClassName} />
      <input
        type="checkbox"
        checked={on}
        disabled={disabled}
        className={"cursor-inherit h-full opacity-0 w-full absolute"}
        onChange={props.onChange}
      />
    </span>
  );
};
