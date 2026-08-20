import React, { useCallback, useContext } from "react";
import { Render2DMouseEvent } from "@client/render2d/event/mouse_event";
import { Renderer } from "@client/render2d/renderer";
import { useUpdatable } from "@hooks/use_updatable";
import { observer } from "mobx-react";

import { TimelineDataPoint } from "../model";
import { timelineContext } from "../view";

import { TimelineRowViewModel } from "./view_model";

export interface TimelineRowProps {
  dataPoints?: TimelineDataPoint[];
  connectorPoints?: TimelineDataPoint[];
  highlightedDataPoints?: TimelineDataPoint[];
  className?: string;
  dataPointWidth?: number;
  dataPointHeight?: number;
  connectorHeight?: number;
  onDataPointClick?: (dataPoint: TimelineDataPoint, e: Render2DMouseEvent) => void;
}

/**
 * Render a timeline row, using the view from the current timeline context.
 *
 * The default color of the data points and connectors in the row uses the current CSS 'color' value.
 */
export const TimelineRow = observer((props: TimelineRowProps) => {
  const { dataPoints, connectorPoints, highlightedDataPoints, className, onDataPointClick } = props;
  const { dataPointWidth, dataPointHeight, connectorHeight } = props;

  const context = useContext(timelineContext);

  if (!context) {
    throw new Error("Cannot have a <TimelineRow> component outside of <TimelineView>");
  }

  const { model, controller } = context;
  const viewModelOpts: ConstructorParameters<typeof TimelineRowViewModel> = [
    model,
    controller,
    dataPoints,
    connectorPoints,
    highlightedDataPoints,
    dataPointWidth,
    dataPointHeight,
    connectorHeight,
    onDataPointClick,
  ];

  // Create single view model instance that updates when props change
  const viewModel = useUpdatable(
    () => new TimelineRowViewModel(...viewModelOpts),
    (instance, newOpts) => instance.update(newOpts),
    viewModelOpts,
  );

  // Track size in px of renderer canvas and update it on the view model
  const onResize = useCallback((width: number, height: number) => {
    viewModel.setCanvasSize({ width: Math.floor(width), height: Math.floor(height) });
  }, []);

  return (
    <div className={className}>
      <div className="relative w-full h-full">
        <Renderer
          engine="svg"
          camera={viewModel.camera}
          scene={viewModel.scene}
          eventHandlers={viewModel.defaultEventHandlers}
          onResize={onResize}
        />
      </div>
    </div>
  );
});
