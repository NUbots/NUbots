import React from "react";
import classNames from "classnames";

import { Icon, IconProps } from "../icon/view";

export type IconButtonProps = React.ButtonHTMLAttributes<HTMLButtonElement> & {
  /** The size of the button */
  size?: "normal" | "large";
  /** The color of the button */
  color?: "default" | "primary" | "transparent";
  /** Props to pass to the `<Icon>` rendered in the button */
  iconProps?: Pick<IconProps, "fill" | "weight" | "flip" | "rotate">;
};

const SizeToClassName = {
  normal: "h-8 w-8 [&_svg]:w-6 [&_svg]:h-6",
  large: "h-10 w-10 [&_svg]:w-8 [&_svg]:h-8",
} as const;

const ColorToClassName = {
  default:
    "shadow-sm bg-white text-gray-600 ring-1 ring-inset ring-gray-300 dark:bg-gray-600 dark:text-white dark:ring-gray-500",
  primary: "dark shadow-sm bg-blue-500 text-white",
  transparent: "bg-transparent text-blue-500 ring-transparent dark:text-blue-500",
} as const;

export function IconButton(props: IconButtonProps) {
  const { size = "normal", color = "default", iconProps, className, children, ...buttonProps } = props;

  return (
    <button
      {...buttonProps}
      className={classNames(
        "relative inline-flex items-center justify-center flex-shrink-0 rounded disabled:opacity-40 group",
        SizeToClassName[size],
        ColorToClassName[color],
        className,
      )}
    >
      <span
        className={classNames(
          "absolute inset-0 rounded group-hover:bg-auto-contrast-1 group-active:bg-auto-contrast-2 pointer-events-none",
        )}
      />
      {typeof children === "string" ? (
        <Icon {...iconProps} className={size === "large" ? "text-[2rem]" : undefined}>
          {children}
        </Icon>
      ) : (
        children
      )}
    </button>
  );
}
