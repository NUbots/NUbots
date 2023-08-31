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
          style.button,
          {
            [style.buttonPrimary]: type === "primary",
            [style.buttonNormal]: type === "normal",
            [style.fullwidth]: fullwidth,
            [style.iconAfterAlignedRight]: iconAfterAlignedRight,
            [style.alignLeft]: textAlign === "left",
            [style.alignRight]: textAlign === "right",
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
