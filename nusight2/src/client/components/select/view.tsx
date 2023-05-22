import React from "react";
import { ReactNode } from "react";
import { action } from "mobx";
import { observable } from "mobx";
import { observer } from "mobx-react";
import OutsideClickHandler from "react-outside-click-handler/esm/OutsideClickHandler";

import { Button } from "../button/view";
import { Dropdown } from "../dropdown/view";
import { Icon } from "../icon/view";

import { SelectOption } from "./option";
import style from "./style.module.css";

export interface Option {
  id: string | number;
  label: string;
  icon?: ReactNode;
}

export type SelectProps = {
  className?: string;
  placeholder: string;
  options: Option[];
  selectedOption?: Option;
  icon?: ReactNode;
  empty?: ReactNode;
  dropDirection?: "up" | "down";
  onChange(option: Option): void;
};

@observer
export class Select extends React.Component<SelectProps> {
  @observable
  private isOpen: boolean = false;
  private removeListeners?: () => void;

  componentDidMount() {
    document.addEventListener("keydown", this.onDocumentKeydown);

    this.removeListeners = () => {
      document.removeEventListener("keydown", this.onDocumentKeydown);
    };
  }

  componentWillUnmount() {
    if (this.removeListeners) {
      this.removeListeners();
    }
  }

  render(): JSX.Element {
    const { className, icon, placeholder, dropDirection, empty, options, selectedOption } = this.props;

    // If one of the options has an icon, show padding for all the options to align their text
    const showOptionIconPadding = options.some((option) => option.icon);

    const button = (
      <Button
        iconBefore={icon}
        iconAfter={<Icon className="-mr-0.5">arrow_drop_down</Icon>}
        textAlign="left"
        iconAfterAlignedRight
        fullwidth
      >
        {selectedOption ? selectedOption.label : placeholder}
      </Button>
    );

    return (
      <OutsideClickHandler onOutsideClick={this.close}>
        <Dropdown
          className={className}
          dropdownToggle={button}
          dropDirection={dropDirection}
          isOpen={this.isOpen}
          isFullwidth={true}
          onToggleClick={this.onToggleClick}
        >
          <div className={style.dropdown}>
            {options.length === 0 && <div className={style.empty}>{empty || "No options"}</div>}
            {options.length > 0 && (
              <div className={style.options}>
                {options.map((option) => {
                  const isSelected = Boolean(selectedOption && selectedOption.id === option.id);
                  return (
                    <SelectOption
                      key={option.id}
                      option={option}
                      icon={option.icon}
                      showIconPadding={showOptionIconPadding}
                      isSelected={isSelected}
                      onSelect={this.onSelect}
                    />
                  );
                })}
              </div>
            )}
          </div>
        </Dropdown>
      </OutsideClickHandler>
    );
  }

  private readonly onDocumentKeydown = (event: KeyboardEvent) => {
    if (this.isOpen && event.key === "Escape") {
      this.close();
    }
  };

  @action.bound
  private onToggleClick() {
    this.toggle();
  }

  @action.bound
  private onSelect(option: Option) {
    this.props.onChange && this.props.onChange(option);
    this.close();
  }

  @action.bound
  private open() {
    if (!this.isOpen) {
      this.isOpen = true;
    }
  }

  @action.bound
  private close() {
    if (this.isOpen) {
      this.isOpen = false;
    }
  }

  @action.bound
  private toggle() {
    this.isOpen = !this.isOpen;
  }
}
