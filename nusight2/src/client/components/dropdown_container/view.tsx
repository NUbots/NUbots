import React from "react";
import { ComponentType } from "react";
import { ReactNode } from "react";
import { action } from "mobx";
import { observable } from "mobx";
import { observer } from "mobx-react";

import { DropdownProps } from "../dropdown/view";
import { Dropdown } from "../dropdown/view";

export type DropdownContainerProps = {
  children?: ReactNode;
  dropdownToggle: ReactNode;
  dropdownPosition?: "left" | "right";
};

enum KeyCode {
  Escape = 27,
}

export const dropdownContainer = (WrappedComponent: ComponentType<DropdownProps> = Dropdown) => {
  // Refer to: https://github.com/Microsoft/TypeScript/issues/7342
  @observer
  class EnhancedDropdown extends React.Component<DropdownContainerProps> {
    private dropdown?: HTMLDivElement;
    @observable private isOpen: boolean = false;
    private removeListeners?: () => void;

    componentDidMount() {
      const onClick = (event: MouseEvent) => this.onDocumentClick(event);
      document.addEventListener("click", onClick);

      const onKeydown = (event: KeyboardEvent) => this.onDocumentKeydown(event);
      document.addEventListener("keydown", onKeydown);

      this.removeListeners = () => {
        document.removeEventListener("click", onClick);
        document.removeEventListener("keydown", onKeydown);
      };
    }

    componentWillUnmount() {
      if (this.removeListeners) {
        this.removeListeners();
      }
    }

    render(): JSX.Element {
      return (
        <WrappedComponent {...this.props} isOpen={this.isOpen} onRef={this.onRef} onToggleClick={this.onToggleClick}>
          {this.props.children}
        </WrappedComponent>
      );
    }

    @action
    private close() {
      if (this.isOpen) {
        this.isOpen = false;
      }
    }

    private onDocumentClick(event: MouseEvent) {
      if (!this.dropdown || !this.isOpen || !isOutsideEl(event.target, this.dropdown)) {
        return;
      }

      this.close();
    }

    private onDocumentKeydown(event: KeyboardEvent) {
      if (!this.dropdown || event.keyCode !== KeyCode.Escape || !this.isOpen) {
        return;
      }

      this.close();
    }

    private onRef = (dropdown: HTMLDivElement) => {
      this.dropdown = dropdown;
    };

    private onToggleClick = () => {
      this.toggle();
    };

    @action
    private open() {
      if (!this.isOpen) {
        this.isOpen = true;
      }
    }

    @action
    private toggle() {
      this.isOpen = !this.isOpen;
    }
  }

  return EnhancedDropdown;
};

function isOutsideEl(target: EventTarget | null, el?: HTMLElement): boolean {
  let current: Node | null = target as Node;
  while (current) {
    if (current === el) {
      return false;
    }
    current = current.parentNode;
  }
  return true;
}
