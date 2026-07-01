import React, { useCallback, useMemo, useState } from "react";
import { lightOrDarkDecorator } from "@components/storybook/color_mode";
import { Meta, StoryObj } from "@storybook/react";
import { action, observable } from "mobx";

import { TimelineHeader } from "../header/timeline_header";
import { TimelineHighlightRange } from "../highlight_range/timeline_highlight_range";
import { TimelineRow } from "../row/timeline_row";
import { TimelineView } from "../view";

const meta: Meta<typeof TimelineHighlightRange> = {
  title: "components/Timeline/HighlightRange",
  decorators: [lightOrDarkDecorator],
};

export default meta;

export const Marker: StoryObj<typeof TimelineHighlightRange> = {
  argTypes: {
    color: { control: "color" },
    hideFlags: { control: "boolean" },
    hideBody: { control: "boolean" },
  },
  args: {
    color: "#FF0000",
    hideFlags: false,
    hideBody: false,
  },

  render: ({ color, hideBody, hideFlags }) => {
    const view = useMemo(() => observable({ centre: BigInt(0), range: BigInt(10e9) }), []);

    const [highlightStart, setHighlightStart] = useState(BigInt(-2e9));
    const [highlightEnd, setHighlightEnd] = useState(BigInt(2e9));
    const [isRangeSelected, setIsRangeSelected] = useState(false);

    const onMouseDown = useCallback(() => setIsRangeSelected(true), []);
    const onMouseUp = useCallback(() => setIsRangeSelected(false), []);
    const onMouseMove = useCallback(
      action((side: "start" | "end", timestamp: bigint) => {
        if (isRangeSelected) {
          if (side === "start") {
            setHighlightStart(timestamp > highlightEnd ? highlightEnd : timestamp);
          } else {
            setHighlightEnd(timestamp < highlightStart ? highlightStart : timestamp);
          }
        }
      }),
      [isRangeSelected],
    );

    return (
      <div className="w-full rounded overflow-hidden bg-auto-surface-0">
        <TimelineView view={view}>
          <TimelineHeader className="h-12 border-b border-auto" />
          <TimelineRow className="h-40" />
          <TimelineHighlightRange
            start={highlightStart}
            end={highlightEnd}
            color={color}
            hideFlags={hideFlags}
            hideBody={hideBody}
            onMouseDown={onMouseDown}
            onMouseUp={onMouseUp}
            onMouseMove={onMouseMove}
          />
        </TimelineView>
      </div>
    );
  },
};
