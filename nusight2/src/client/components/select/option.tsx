import React, { ReactNode } from "react";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import style from "./style.module.css";
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
        className={classNames([className, style.option, isSelected ? style.optionSelected : ""])}
        onClick={this.onSelect}
      >
        {showIconPadding || icon ? <span className={style.optionIcon}>{icon}</span> : null}
        {option.label}
      </div>
    );
  }

  @action.bound
  private onSelect() {
    this.props.onSelect(this.props.option);
  }
}
