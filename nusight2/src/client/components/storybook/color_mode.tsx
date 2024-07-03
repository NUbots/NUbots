import React, { useCallback, useState } from "react";
import classNames from "classnames";

import { Icon } from "../icon/view";

export interface ModeProps {
  children: React.ReactNode;
  className?: string;
  onToggle?: () => void;
}

function ColorMode({ mode, children, className, onToggle }: ModeProps & { mode: "light" | "dark" }) {
  const toggleable = typeof onToggle === "function";

  const indicatorClassName =
    "absolute top-0 left-0 bg-auto-contrast-1 py-1.5 px-2.5 rounded-br border-r border-b border-auto flex text-auto-primary";

  return (
    <div
      className={classNames("rounded-lg overflow-hidden ", mode === "dark" ? "dark" : "border border-auto", className)}
    >
      <div className="h-full relative px-14 py-10 overflow-hidden bg-auto-surface-2 text-auto-primary">
        {toggleable ? (
          <button
            className={classNames(indicatorClassName, toggleable ? "hover:bg-auto-contrast-2" : "")}
            title={toggleable ? `Switch to ${mode === "dark" ? "light" : "dark"} mode` : undefined}
            onClick={toggleable ? onToggle : undefined}
          >
            <Icon size="20">{mode === "dark" ? "light_mode" : "dark_mode"}</Icon>
          </button>
        ) : (
          <span className={indicatorClassName}>
            <Icon size="20">{mode === "dark" ? "dark_mode" : "light_mode"}</Icon>
          </span>
        )}
        {children}
      </div>
    </div>
  );
}

function LightMode({ children, className }: ModeProps) {
  return (
    <ColorMode mode="light" className={className}>
      {children}
    </ColorMode>
  );
}

function DarkMode({ children, className }: ModeProps) {
  return (
    <ColorMode mode="dark" className={className}>
      {children}
    </ColorMode>
  );
}

export interface LightAndDarkOpts {
  className?: string;
  horizontal?: boolean;
}

export function LightAndDark({ children, opts }: { children: () => React.ReactElement; opts?: LightAndDarkOpts }) {
  const lightClassName = opts?.horizontal ? "border-r-0 rounded-r-none" : "border-b-0 rounded-b-none";
  const darkClassName = opts?.horizontal ? "rounded-l-none" : "rounded-t-none";

  return (
    <div className={classNames("flex h-full", { "flex-col": !opts?.horizontal })}>
      <LightMode className={classNames(lightClassName, opts?.className)}>{children()}</LightMode>
      <DarkMode className={classNames(darkClassName, opts?.className)}>{children()}</DarkMode>
    </div>
  );
}

export interface LightOrDarkOpts {
  className?: string;
  reRenderOnToggle?: boolean;
}

export function LightOrDark({ children, opts }: { children: () => React.ReactElement; opts?: LightOrDarkOpts }) {
  const [mode, setMode] = useState<"light" | "dark">("light");
  const [renderKey, setRenderKey] = useState(0);

  const toggleMode = useCallback(() => {
    setMode((mode) => (mode === "light" ? "dark" : "light"));
    if (opts?.reRenderOnToggle) {
      setRenderKey((key) => key + 1);
    }
  }, []);

  const ReRenderableContent = () => {
    return <>{children()}</>;
  };

  return (
    <div className="h-full">
      <ColorMode mode={mode} className={classNames(opts?.className)} onToggle={toggleMode}>
        {opts?.reRenderOnToggle ? <ReRenderableContent key={renderKey} /> : children()}
      </ColorMode>
    </div>
  );
}

/** Decorator to render a story in both light and dark modes */
export function lightAndDarkDecorator(story?: () => React.ReactElement): React.ReactElement;

/** Decorator to render a story in both light and dark modes with additional options */
export function lightAndDarkDecorator(opts?: LightAndDarkOpts): () => React.ReactElement;

/**
 * Decorator to render a story in both light and dark modes.
 *
 * Can be used directly by adding the function to the story decorators, or can
 * be called to add options to the decorator.
 */
export function lightAndDarkDecorator(opts?: (() => React.ReactElement) | LightAndDarkOpts) {
  if (typeof opts === "function") {
    return <LightAndDark>{opts}</LightAndDark>;
  }

  return (story: () => React.ReactElement) => <LightAndDark opts={opts}>{story}</LightAndDark>;
}

/** Decorator to render a story in light or dark mode with a toggle */
export function lightOrDarkDecorator(story?: () => React.ReactElement): React.ReactElement;

/** Decorator to render a story in light or dark mode with a toggle with additional options */
export function lightOrDarkDecorator(opts?: LightOrDarkOpts): () => React.ReactElement;

/**
 * Decorator to render a story in light or dark mode with a toggle.
 *
 * Can be used directly by adding the function to the story decorators, or can
 * be called to add options to the decorator.
 */
export function lightOrDarkDecorator(opts?: (() => React.ReactElement) | LightOrDarkOpts) {
  if (typeof opts === "function") {
    return <LightOrDark>{opts}</LightOrDark>;
  }

  return (story: () => React.ReactElement) => <LightOrDark opts={opts}>{story}</LightOrDark>;
}
