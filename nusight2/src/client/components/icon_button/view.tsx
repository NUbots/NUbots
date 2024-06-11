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
    "shadow-sm bg-white text-gray-500 ring-1 ring-inset ring-gray-300 enabled:hover:bg-gray-50 enabled:active:bg-gray-200 dark:bg-gray-700 dark:text-white dark:ring-gray-600 dark:enabled:hover:bg-gray-600 dark:enabled:hover:ring-gray-500 dark:enabled:active:bg-gray-700",
  primary: "shadow-sm bg-blue-500 text-white enabled:hover:bg-blue-400 enabled:active:bg-blue-600",
  transparent:
    "bg-transparent text-white ring-transparent enabled:hover:bg-black/10 enabled:active:bg-black/20 dark:enabled:hover:bg-white/10 dark:enabled:active:bg-black/20",
  semitransparent:
    "bg-black/60 text-white ring-transparent enabled:hover:bg-black/60 enabled:active:bg-black/40 enabled:hover:bg-black/50 ",
} as const;

export function IconButton(props: IconButtonProps) {
  const { size = "normal", color = "default", iconProps, className, children, ...buttonProps } = props;

  return (
    <button
      {...buttonProps}
      className={classNames(
        "inline-flex items-center justify-center flex-shrink-0 rounded disabled:opacity-40",
        SizeToClassName[size],
        ColorToClassName[color],
        className,
      )}
    >
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
