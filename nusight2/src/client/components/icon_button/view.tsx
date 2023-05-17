import React from "react";
import classNames from "classnames";

import { Icon, IconProps } from "../icon/view";

export interface IconButtonProps {
  /** The icon name */
  children: React.ReactNode;
  /** The title (tooltip) of the button */
  title?: string;
  /** The size of the button */
  size?: "normal" | "large";
  /** The color of the button */
  color?: "default" | "primary" | "transparent";
  /** Props to pass to the `<Icon>` rendered in the button */
  iconProps?: Pick<IconProps, "fill" | "weight" | "flip" | "rotate">;
  /** Whether the button is disabled */
  disabled?: boolean;
  className?: string;
  onClick?: () => void;
}

const SizeToClassName = {
  normal: "h-8 w-8 [&_svg]:w-6 [&_svg]:h-6",
  large: "h-10 w-10 [&_svg]:w-8 [&_svg]:h-8",
} as const;

const ColorToClassName = {
  default:
    "shadow-sm bg-white text-gray-600 ring-1 ring-inset ring-gray-300 enabled:hover:bg-gray-50 enabled:active:bg-gray-200 dark:bg-slate-700 dark:text-white dark:ring-transparent dark:enabled:hover:bg-slate-600 dark:enabled:active:bg-slate-800",
  primary: "shadow-sm bg-nusight-500 text-white enabled:hover:bg-nusight-400 enabled:active:bg-nusight-600",
  transparent:
    "bg-transparent text-nusight-500 ring-transparent enabled:hover:bg-black/10 enabled:active:bg-black/20 dark:text-nusight-500 dark:enabled:hover:bg-white/10 dark:enabled:active:bg-black/20",
} as const;

export function IconButton(props: IconButtonProps) {
  const { children, title, size = "normal", color = "default", iconProps, className, disabled, onClick } = props;

  return (
    <button
      title={title}
      className={classNames(
        "inline-flex items-center justify-center rounded disabled:opacity-40",
        SizeToClassName[size],
        ColorToClassName[color],
        className,
      )}
      disabled={disabled}
      onClick={onClick}
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
