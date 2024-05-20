import React from "react";
import { ChangeEvent } from "react";
import classNames from "classnames";

export interface SwitchProps {
  on: boolean;
  disabled?: boolean;

  onChange(event: ChangeEvent<HTMLInputElement>): void;
}

// TODO: Fix vertical alignment of switch

export const Switch = (props: SwitchProps) => {
  const { disabled, on } = props;
  const trackClassName = classNames("rounded-full h-[1.1em] w-inherit relative", {
    ["bg-gray-400 dark:bg-gray-600"]: !on,
    ["bg-blue-600 opacity-50"]: on,
  });
  const thumbClassName = classNames(
    "rounded-full shadow-md h-[1.6em] left-0 absolute w-[1.6em] top-0 transition-transform duration-250 ease-in-out",
    {
      ["bg-gray-100 dark:bg-gray-500"]: !on,
      ["bg-blue-600 translate-x-[1.3em]"]: on,
    },
  );
  return (
    <span className={"flex items-center cursor-pointer h-[1.6em] relative w-[2.85em]"}>
      <span className={trackClassName} />
      <span role="thumb" className={thumbClassName} />
      <input
        type="checkbox"
        checked={on}
        disabled={disabled}
        className={"cursor-inherit h-inherit opacity-0 w-inherit absolute"}
        onChange={props.onChange}
      />
    </span>
  );
};
