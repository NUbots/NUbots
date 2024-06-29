import React, { ReactNode } from "react";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Option } from "./view";

export type SelectOptionProps = {
  className?: string;
  option: Option;
  showIconPadding: boolean;
  icon?: ReactNode;
  isSelected: boolean;
  onSelect(option: Option): void;
};

@observer
export class SelectOption extends React.Component<SelectOptionProps> {
  render(): JSX.Element {
    const { className, option, showIconPadding, icon, isSelected } = this.props;

    return (
      <div
        className={classNames([
          className,
          "items-center p-2 text left cursor-pointer",
          isSelected
            ? "bg-blue-600 text-white"
            : "hover:bg-auto-contrast-2",
        ])}
        onClick={this.onSelect}
      >
        {showIconPadding || icon ? (
          <span className="w-5 h-5 mr-2">
            <div className="w-full h-full inline">{icon}</div>
          </span>
        ) : null}
        {option.label}
      </div>
    );
  }

  @action.bound
  private onSelect() {
    this.props.onSelect(this.props.option);
  }
}
