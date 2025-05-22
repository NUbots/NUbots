import { forwardRef, useContext, useEffect, useImperativeHandle } from "react";
import React from "react";
import classNames from "classnames";

import { resizeContainerContext, resizePanelContext } from "./resize_container";

export type ResizePanelState = "default" | "minimized" | "maximized";

export interface ResizePanelRef {
  setState: (state: ResizePanelState) => void;
}

export interface ResizePanelProps {
  initialRatio?: number;
  minSize?: number;
  maxSize?: number;
  className?: string;
  children?: React.ReactNode;
  onStateChange?: (state: ResizePanelState) => void;
}

export const ResizePanel = forwardRef(function ResizePanel(
  props: ResizePanelProps,
  ref?: React.ForwardedRef<ResizePanelRef>,
) {
  const { className, children, onStateChange } = props;
  const { index, size, state } = useContext(resizePanelContext);
  const { setPanelState } = useContext(resizeContainerContext);

  useEffect(() => {
    onStateChange?.(state);
  }, [state]);

  // Expose handle to parent with control methods
  useImperativeHandle(
    ref,
    () => ({
      setState: (state) => {
        setPanelState(index, state);
      },
      state,
    }),
    [index, state, setPanelState],
  );

  return (
    <div className={classNames("h-full overflow-hidden", className)} style={{ flex: `1 0 ${size}%` }}>
      {children}
    </div>
  );
});
