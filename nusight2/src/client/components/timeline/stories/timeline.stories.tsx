import React, { useMemo, useState } from "react";
import { IconButton } from "@components/icon_button/view";
import { lightOrDarkDecorator } from "@components/storybook/color_mode";
import { clamp } from "@shared/math/clamp";
import { Meta, StoryObj } from "@storybook/react";
import classNames from "classnames";
import { action, observable } from "mobx";

import { TimelineHeader } from "../header/timeline_header";
import { TimelineMarker } from "../marker/timeline_marker";
import { TimelineDataPoint } from "../model";
import { TimelineRow } from "../row/timeline_row";
import { TimelineView } from "../view";

type StoryComponent = React.FunctionComponent<{
  headerHeight: number;
  rowHeight: number;
  dataPointWidth: number;
  dataPointHeight: number;
  connectorHeight: number;
}>;

const meta: Meta<StoryComponent> = {
  title: "components/Timeline",
  decorators: [lightOrDarkDecorator],
  argTypes: {
    headerHeight: { control: { type: "range", min: 1, max: 200, step: 1 } },
    rowHeight: { control: { type: "range", min: 1, max: 200, step: 1 } },
    dataPointWidth: { control: { type: "range", min: 1, max: 200, step: 1 } },
    dataPointHeight: { control: { type: "range", min: 1, max: 200, step: 1 } },
    connectorHeight: { control: { type: "range", min: 1, max: 200, step: 1 } },
  },
  args: {
    headerHeight: 60,
  },
};

const currentTimeNanos = BigInt(Math.floor(Date.now() * 1e6));

export default meta;

function createTimestampRange(start: bigint, length: number, step: bigint) {
  function variance() {
    return BigInt(Math.floor(Math.random() * Number(step / 2n) - Number(step / 4n)));
  }

  return Array(length)
    .fill(0n)
    .map((_, i) => start + step * BigInt(i) + variance());
}

function createPackets(opts: { frequency: number; time: number }) {
  const stepSize = Math.floor(1e9 / opts.frequency);
  const length = Math.floor(opts.time / stepSize);
  const step = BigInt(stepSize);

  return createTimestampRange(currentTimeNanos, length, step).map((timestamp) => ({ timestamp }));
}

function fakeTimelineContent(): { dataPoints: TimelineDataPoint[]; className?: string }[] {
  const footageLength = 100000000000;
  return [
    {
      dataPoints: [
        { timestamp: currentTimeNanos + BigInt(1e9), color: "#bb1cd4" },
        { timestamp: currentTimeNanos + BigInt(2e9), color: "#bb1cd4" },
        { timestamp: currentTimeNanos + BigInt(5e9), color: "#eb8334" },
        { timestamp: currentTimeNanos + BigInt(6e9), color: "#bb1cd4" },
        { timestamp: currentTimeNanos + BigInt(6.5e9), color: "#eb4034" },
        { timestamp: currentTimeNanos + BigInt(9e9), color: "#1c99d4" },
      ],
    },
    { dataPoints: createPackets({ frequency: 237.1, time: footageLength / 2 }), className: "text-red-500" },
    { dataPoints: createPackets({ frequency: 10, time: footageLength }), className: "text-green-500" },
    { dataPoints: createPackets({ frequency: 2, time: footageLength * 0.75 }), className: "text-blue-500" },
  ];
}

export const Default: StoryObj<StoryComponent> = {
  render: () => {
    const view = useMemo(() => observable({ centre: currentTimeNanos, range: BigInt(10e9) }), []);
    const content = useMemo(() => fakeTimelineContent(), []);

    const [highlightedDataPoint, setHighlightedDataPoint] = useState<TimelineDataPoint | null>(null);

    return (
      <div className="flex flex-col gap-2">
        <div className="flex justify-end gap-1">
          <IconButton onClick={action(() => (view.range += view.range / 5n))}>zoom_out</IconButton>
          <IconButton onClick={action(() => (view.range -= view.range / 6n))}>zoom_in</IconButton>
        </div>
        <TimelineView view={view} className="flex flex-col w-full rounded overflow-hidden bg-auto-surface-0">
          <TimelineHeader className="h-12 border-b border-auto" />
          {content.map(({ dataPoints, className }, i) => (
            <TimelineRow
              key={i}
              className={classNames("h-8 border-b border-auto", className)}
              dataPoints={dataPoints}
              highlightedDataPoints={highlightedDataPoint ? [highlightedDataPoint] : undefined}
              onDataPointClick={setHighlightedDataPoint}
            />
          ))}
        </TimelineView>
      </div>
    );
  },
};

export const RangeLimit: StoryObj<StoryComponent> = {
  render: () => {
    const viewMin = 0n;
    const viewMax = BigInt(10e9);
    const minRange = BigInt(0.1e9);
    const view = useMemo(() => observable({ centre: BigInt(5e9), range: BigInt(10e9) }), []);

    return (
      <div className="w-full rounded overflow-hidden bg-auto-surface-0">
        <TimelineView
          view={view}
          onViewCentreChange={(viewCentre) => {
            const min = viewMin + view.range / 2n;
            const max = viewMax - view.range / 2n;
            view.centre = clamp(viewCentre, min, max);
          }}
          onViewRangeChange={(viewRange) => {
            view.range = clamp(viewRange, minRange, viewMax - viewMin);
          }}
        >
          <TimelineHeader className="h-12 border-b border-auto" />
          <TimelineRow className="h-40" />
        </TimelineView>
      </div>
    );
  },
};

export const TimeInput: StoryObj<StoryComponent> = {
  render: () => {
    const view = useMemo(() => observable({ centre: BigInt(5e9), range: BigInt(10e9) }), []);

    return (
      <div className="w-full rounded overflow-hidden bg-auto-surface-0">
        <TimelineView view={view}>
          <TimelineHeader className="h-12 border-b border-auto" />
          <TimelineRow className="h-40" />
          <DraggableMarker startTimestamp={BigInt(3e9)} color="#0ea5e9" />
          <DraggableMarker startTimestamp={BigInt(7e9)} color="#e11d48" />
        </TimelineView>
      </div>
    );
  },
};

function DraggableMarker(props: { startTimestamp: bigint; color: string }) {
  const { startTimestamp, color } = props;
  const [timestamp, setTimestamp] = useState(startTimestamp);
  const [isDragging, setIsDragging] = useState(false);
  return (
    <TimelineMarker
      timestamp={timestamp}
      color={color}
      onMouseDown={() => setIsDragging(true)}
      onMouseUp={() => setIsDragging(false)}
      onMouseMove={(time) => {
        if (isDragging) {
          setTimestamp(time);
        }
      }}
      clampToView
    />
  );
}
