import { useCallback, useContext } from "react";
import React from "react";
import { Renderer } from "@client/render2d/renderer";
import { useUpdatable } from "@hooks/use_updatable";
import classNames from "classnames";
import { observer } from "mobx-react";

import { timelineContext } from "../view";

import { TimelineHeaderViewModel } from "./view_model";

export type HeaderInput = (time: bigint) => void;

export interface TimelineHeaderProps {
  className?: string;
  onPointerDown?: HeaderInput;
  onPointerMove?: HeaderInput;
  onPointerUp?: HeaderInput;
  onMouseEnter?: () => void;
  onMouseLeave?: () => void;
}

export const TimelineHeader = observer(function TimelineHeader(props: TimelineHeaderProps) {
  const { onPointerDown, onPointerMove, onPointerUp, onMouseEnter, onMouseLeave, className } = props;
  const context = useContext(timelineContext);

  if (!context) {
    throw new Error("Cannot have a <TimelineHeader> component outside of a <TimelineView>");
  }

  const { model, controller } = context;
  const viewModelOpts: ConstructorParameters<typeof TimelineHeaderViewModel> = [
    model,
    controller,
    onPointerDown,
    onPointerMove,
    onPointerUp,
  ];

  // Create single view model instance that updates when props change
  const viewModel = useUpdatable(
    () => new TimelineHeaderViewModel(...viewModelOpts),
    (instance, newOpts) => instance.update(newOpts),
    [model, controller, onPointerDown, onPointerMove, onPointerUp],
  );

  // Track size in px of renderer canvas and update it on the view model
  const onResize = useCallback((width: number, height: number) => {
    viewModel.setCanvasSize({ width: Math.floor(width), height: Math.floor(height) });
  }, []);

  return (
    <div
      className={classNames("relative min-h-[2rem]", className)}
      onMouseEnter={onMouseEnter}
      onMouseLeave={onMouseLeave}
    >
      <Renderer
        engine="svg"
        camera={viewModel.camera}
        scene={viewModel.scene}
        eventHandlers={viewModel.eventHandlers}
        onResize={onResize}
      />
    </div>
  );
});
