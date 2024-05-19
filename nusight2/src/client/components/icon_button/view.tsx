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
    "shadow-sm bg-gray-100 text-gray-900 ring-1 ring-inset ring-gray-300 hover:enabled:bg-gray-300 dark:bg-gray-700 dark:hover:enabled:bg-gray-600 dark:text-gray-100 dark:ring-transparent",
  primary: "shadow-sm bg-blue-600 text-white enabled:hover:bg-blue-700 dark:enabled:hover:bg-blue-500 enabled:active:bg-blue-700",
  transparent:
    "bg-transparent ring-transparent enabled:hover:bg-black/10 enabled:active:bg-black/20 dark:enabled:hover:bg-white/10 dark:enabled:active:bg-black/20",
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
