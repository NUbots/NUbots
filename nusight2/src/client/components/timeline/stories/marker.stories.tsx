import React, { useMemo } from "react";
import { lightOrDarkDecorator } from "@components/storybook/color_mode";
import { Meta, StoryObj } from "@storybook/react";
import { observable } from "mobx";

import { TimelineHeader } from "../header/timeline_header";
import { TimelineMarker } from "../marker/timeline_marker";
import { TimelineRow } from "../row/timeline_row";
import { TimelineView } from "../view";

const meta: Meta<typeof TimelineMarker> = {
  title: "components/Timeline/Marker",
  decorators: [lightOrDarkDecorator],
  argTypes: {
    text: { control: { type: "text" } },
    color: { control: { type: "color" } },
    scale: { control: { type: "range", min: 0.1, max: 2, step: 0.05 } },
    clampToView: { control: { type: "boolean" } },
    hideTail: { control: { type: "boolean" } },
  },
  args: {
    color: "#FF0000",
    scale: 1,
    clampToView: false,
    hideTail: false,
  },
};

export default meta;

export const Marker: StoryObj<typeof TimelineMarker> = {
  render: ({ text, color, scale, clampToView, hideTail }) => {
    const view = useMemo(() => observable({ centre: BigInt(0), range: BigInt(10e9) }), []);
    return (
      <div className="w-full rounded overflow-hidden bg-auto-surface-0">
        <TimelineView view={view}>
          <TimelineHeader className="h-12 border-b border-auto" />
          <TimelineRow className="h-40" />
          <TimelineMarker
            timestamp={0n}
            text={text}
            color={color}
            scale={scale}
            clampToView={clampToView}
            hideTail={hideTail}
          />
        </TimelineView>
      </div>
    );
  },
};
