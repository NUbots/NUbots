import React from "react";

import { FocusContainer } from "../focus_container/focus_container";
import { Icon } from "../icon/view";

export type ModalProps = {
  children: React.ReactNode;
  closeWith?: {
    escapeKey?: boolean;
    closeButton?: boolean;
    backdropClick?: boolean;
  };
  onClose(): void;
};

const closeWithDefaults = {
  escapeKey: true,
  closeButton: true,
  backdropClick: false,
};

/** Shows a modal dialog (i.e. one that prevents interaction with content outside it until closed) */
export const Modal = (props: ModalProps) => {
  // Merge the defaults with the `closeWith` prop, allowing the prop to override defaults
  const closeWith = Object.assign({}, closeWithDefaults, props.closeWith);

  return (
    <div
      className="fixed flex items-center justify-center bg-[rgba(0,0,0,0.5)] h-full w-full top-0 left-0 z-10"
      tabIndex={-1}
      onClick={
        closeWith.backdropClick
          ? (event) => {
              // Close the modal when the backdrop is clicked, making sure that
              // the click is on the backdrop itself, not a child element
              if (event.target === event.currentTarget) {
                props.onClose();
              }
            }
          : undefined
      }
      onKeyDown={
        closeWith.escapeKey
          ? (event) => {
              // Close the modal when the escape key is pressed
              if (event.key === "Escape") {
                event.preventDefault();
                props.onClose();
              }
            }
          : undefined
      }
    >
      <FocusContainer>
        <div className="bg-auto-surface-2 text-auto-primary rounded shadow-lg relative">
          {closeWith.closeButton ? (
            <button
              className="absolute right-0 top-0 inline-flex items-center p-2 rounded text-icon dark:text-white hover:bg-gray-100 hover:text-black dark:hover:bg-white/10 focus:bg-gray-100 focus:text-black overflow-hidden"
              data-focus-container-close-button
              onClick={props.onClose}
            >
              <Icon>close</Icon>
            </button>
          ) : null}
          {props.children}
        </div>
      </FocusContainer>
    </div>
  );
};
