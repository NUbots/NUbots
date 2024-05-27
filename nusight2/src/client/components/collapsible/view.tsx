import React from "react";
import classNames from "classnames";
import { observer } from "mobx-react";
import { CSSTransition } from "react-transition-group";

import style from "./style.module.css";

export type CollapsibleProps = {
  className?: string;
  open: boolean;
  header?: React.ReactNode;
  children?: React.ReactNode;
  animate?: boolean;
  onToggle?(): void;
};

export const Collapsible = observer((props: CollapsibleProps) => {
  const { open, className, header, children, onToggle, animate = true } = props;
  return (
    <div>
      {header && (
        <button className={style.collapsibleHeader} onClick={onToggle}>
          {header}
        </button>
      )}
      <CSSTransition
        in={open}
        timeout={200}
        enter={animate}
        exit={animate}
        onEnter={onEnter}
        onEntering={onEntering}
        onEntered={onEntered}
        onExit={onExit}
        onExiting={onExiting}
        unmountOnExit
      >
        <div className={style.collapsibleTransition}>
          <div className={classNames(style.collapsibleBody, className)}>{children}</div>
        </div>
      </CSSTransition>
    </div>
  );
});

function onEnter(node: HTMLElement) {
  // Initialize max-height to a numeric value for the enter transition
  node.style.maxHeight = "0";

  // Hide the overflowing content during the transition
  node.dataset.originalOverflow = node.style.overflow;
  node.style.overflow = "hidden";
}

function onEntering(node: HTMLElement) {
  // Enter to the element's natural scroll height
  node.style.maxHeight = node.scrollHeight + "px";
}

function onEntered(node: HTMLElement) {
  // Enter transition done: remove max-height to allow the element to grow
  node.style.maxHeight = "none";

  // Show overflowing content again
  node.style.overflow = node.dataset.originalOverflow ?? "";
}

function onExit(node: HTMLElement) {
  // Restore max-height for the exit transition
  node.style.maxHeight = node.scrollHeight + "px";

  // Hide the overflowing content during the transition
  node.dataset.originalOverflow = node.style.overflow;
  node.style.overflow = "hidden";
}

function onExiting(node: HTMLElement) {
  // Exit to a max-height of 0
  node.style.maxHeight = "0";
}
