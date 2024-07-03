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
    "shadow-sm bg-white text-gray-600 ring-1 ring-inset ring-gray-300 enabled:hover:bg-gray-50 enabled:active:bg-gray-200 dark:bg-slate-800 dark:text-white dark:ring-slate-700 dark:enabled:hover:bg-slate-700 dark:enabled:hover:ring-slate-600 dark:enabled:active:bg-slate-800",
  primary: "shadow-sm bg-nusight-500 text-white enabled:hover:bg-nusight-400 enabled:active:bg-nusight-600",
  transparent:
    "bg-transparent text-nusight-500 ring-transparent enabled:hover:bg-black/10 enabled:active:bg-black/20 dark:text-nusight-500 dark:enabled:hover:bg-white/10 dark:enabled:active:bg-black/20",
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
