import React from "react";
import { ReactNode } from "react";
import classNames from "classnames";

import style from "./style.module.css";

export type ButtonProps = {
  type?: "normal" | "primary";
  fullwidth?: boolean;
  textAlign?: "left" | "center" | "right";
  disabled?: boolean;
  iconBefore?: ReactNode;
  iconAfter?: ReactNode;
  iconAfterAlignedRight?: boolean;
  className?: string;
  children?: any;
  onClick?(): void;
};

export class Button extends React.PureComponent<ButtonProps> {
  render() {
    const {
      type = "normal",
      textAlign = "center",
      fullwidth,
      disabled,
      iconBefore,
      iconAfter,
      iconAfterAlignedRight,
      className,
      children,
      onClick,
    } = this.props;
    return (
      <button
        onClick={onClick}
        disabled={disabled}
        className={classNames(
          "inline-flex h-[32px] min-w-[64px] px-[12px] items-center font-inherit rounded-md text-md justify-center border disabled:opacity-50",
          {
            ["bg-blue-200 border-blue-400 text-white " + (disabled ? "" : "hover:bg-blue-300")]: type === "primary",
            ["bg-gray-100 border-gray-300 " + (disabled ? "" : "hover:bg-gray-200")]: type === "normal",
            ["w-full"]: fullwidth,
            ["ml_auto"]: iconAfterAlignedRight,
            ["justify-start"]: textAlign === "left",
            ["justify-end"]: textAlign === "right",
          },
          className,
        )}
      >
        {iconBefore && <span className={style.iconBefore}>{iconBefore}</span>}
        {children}
        {iconAfter && (
          <span
            className={classNames(style.iconAfter, {
              [style.iconAfterAlignedRight]: iconAfterAlignedRight,
            })}
          >
            {iconAfter}
          </span>
        )}
      </button>
    );
  }
}
