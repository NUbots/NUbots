import React from "react";
import classNames from "classnames";
import { observer } from "mobx-react";
import { CSSTransition } from "react-transition-group";

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
        <button
          className={
            "appearance-none outline-none bg-gray-300 dark:bg-gray-700 w-full py-[8px] px-[12px] text-left font-inherit hover:bg-gray-350 dark:hover:bg-gray-650"
          }
          onClick={onToggle}
        >
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
        <div className={"overflow-hidden  transition-all duration-200 ease"}>
          <div
            className={classNames(
              "box-border border border-gray-300 dark:border-gray-700 py-[12px] px-[8px]",
              className,
            )}
          >
            {children}
          </div>
        </div>
      </CSSTransition>
    </div>
  );
});

function onEnter(node: HTMLElement) {
  // Initialize max-height to a numeric value for the enter transition
  node.style.maxHeight = "0";
}

function onEntering(node: HTMLElement) {
  // Enter to the element's natural scroll height
  node.style.maxHeight = node.scrollHeight + "px";
}

function onEntered(node: HTMLElement) {
  // Enter transition done: remove max-height to allow the element to grow
  node.style.maxHeight = "none";
}

function onExit(node: HTMLElement) {
  // Restore max-height for the exit transition
  node.style.maxHeight = node.scrollHeight + "px";
}

function onExiting(node: HTMLElement) {
  // Exit to a max-height of 0
  node.style.maxHeight = "0";
}
