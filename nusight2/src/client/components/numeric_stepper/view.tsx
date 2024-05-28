import React, { useState } from "react";
import classNames from "classnames";

import { Icon } from "../icon/view";

export type NumericStepperProps = {
  /** The stepper's current value */
  value: number;
  /** A callback that is called when the stepper's value changes */
  onChange: (value: number) => void;
  /** A function that formats the stepper's value for display */
  formatValue?: (value: number) => string;
  /** The stepper's label */
  label?: string;
  /** The minimum value that the stepper can be set to */
  min?: number;
  /** The maximum value that the stepper can be set to */
  max?: number;
  /** The amount that the stepper's value changes when the increase or decrease buttons are clicked */
  step?: number;
  /** The title (tooltip) of the increase button */
  increaseButtonTitle?: string;
  /** The title (tooltip) of the decrease button */
  decreaseButtonTitle?: string;
  /** The title (tooltip) of the reset button */
  resetButtonTitle?: string;
  /** The width of the value display */
  valueWidth?: "normal" | "wide" | "wider";
} & (
  | {
      /** Whether the stepper can be reset to its default value */
      canReset?: true;
      /** The default value that the stepper should be reset to */
      defaultValue: number;
    }
  | {
      /** Whether the stepper can be reset to its default value */
      canReset?: false;
    }
);

const ValueWidthToClassName = {
  normal: "w-[3rem]",
  wide: "w-[4.5rem]",
  wider: "w-[5.5rem]",
} as const;

export function NumericStepper(props: NumericStepperProps) {
  const {
    value,
    label,
    min = -Infinity,
    max = Infinity,
    step = 1,
    onChange,
    formatValue,
    valueWidth = "normal",
  } = props;

  const [isEditing, setIsEditing] = useState(false);
  const [inputValue, setInputValue] = useState(String(value));

  function setValue(newValue: number) {
    const clampedValue = Math.min(Math.max(newValue, min), max);
    onChange(clampedValue);
  }

  function startEditing() {
    setInputValue(String(value));
    setIsEditing(true);
  }

  function endEditing() {
    const newValue = Number(inputValue);

    if (!Number.isNaN(newValue)) {
      setValue(newValue);
    }

    setIsEditing(false);
  }

  return (
    <div className="inline-flex items-center gap-1 rounded p-1 shadow-sm start bg-gray-200 ring-1 ring-inset ring-gray-300 dark:bg-slate-900 dark:ring-slate-700">
      {label ? <div className="pl-1.5 pr-2 text-sm leading-none dark:text-white">{label}</div> : null}

      <NumericStepperButton
        title={props.decreaseButtonTitle ?? "Decrease"}
        icon="remove"
        disabled={value <= min}
        onClick={() => {
          setValue(value - step);
        }}
      />

      <div
        className={classNames(
          "px-0.5 text-sm text-center flex items-center justify-center dark:text-white",
          ValueWidthToClassName[valueWidth],
        )}
        onClick={startEditing}
      >
        {isEditing ? (
          <input
            autoFocus
            inputMode="numeric"
            value={inputValue}
            onChange={(event) => setInputValue(event.target.value)}
            onFocus={(event) => (event.target as HTMLInputElement).select()}
            onBlur={endEditing}
            onKeyDown={(event) => {
              if (event.key === "Enter") {
                endEditing();
              }
            }}
            className="w-full text-center rounded-sm bg-transparent px-1 outline-none focus:ring-2 focus:ring-nusight-500"
          />
        ) : (
          <span className="inline-block overflow-hidden text-ellipsis">{formatValue ? formatValue(value) : value}</span>
        )}
      </div>

      <NumericStepperButton
        title={props.increaseButtonTitle ?? "Increase"}
        icon="add"
        disabled={value >= max}
        onClick={() => {
          setValue(value + step);
        }}
      />

      {props.canReset ? (
        <NumericStepperButton
          title={props.resetButtonTitle ?? "Reset"}
          icon="undo"
          disabled={value === props.defaultValue}
          onClick={() => {
            setValue(props.defaultValue);
          }}
        />
      ) : null}
    </div>
  );
}

interface NumericStepperButtonProps {
  icon: string;
  onClick?: () => void;
  title?: string;
  disabled?: boolean;
}

function NumericStepperButton(props: NumericStepperButtonProps) {
  return (
    <button
      title={props.title}
      className="rounded-sm inline-flex items-center disabled:opacity-60 justify-center w-6 h-6 bg-white text-gray-700 enabled:hover:text-nusight-600 enabled:active:bg-gray-100 dark:bg-slate-700 dark:p-0.5 dark:text-white/95 dark:enabled:hover:text-nusight-400 dark:enabled:active:bg-slate-800"
      disabled={props.disabled}
      onClick={props.onClick}
    >
      <Icon className="text-lg/none">{props.icon}</Icon>
    </button>
  );
}
