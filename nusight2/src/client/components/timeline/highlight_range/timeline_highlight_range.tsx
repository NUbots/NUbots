import { useContext, useEffect } from "react";
import { usePropertyChange } from "@hooks/use_property_change";
import { action, observable } from "mobx";
import { observer } from "mobx-react";

import { HighlightRangeModel } from "../model";
import { timelineContext } from "../view";

export type TimelineHighlightRangeProps = HighlightRangeModel;

export const TimelineHighlightRange = observer(function TimelineHighlightRange(props: TimelineHighlightRangeProps) {
  const context = useContext(timelineContext);
  if (!context) {
    throw new Error("Cannot have a <HighlightRange> component outside of a <TimelineView>");
  }

  const { model } = context;

  const highlightRangeModel = usePropertyChange<TimelineHighlightRangeProps, HighlightRangeModel>(
    () =>
      observable({
        start: props.start,
        end: props.end,
        color: props.color,
        borderColor: props.borderColor,
        hideFlags: props.hideFlags,
        hideBody: props.hideBody,
        onMouseDown: props.onMouseDown,
        onMouseUp: props.onMouseUp,
        onMouseMove: props.onMouseMove,
        onMouseEnter: props.onMouseEnter,
        onMouseLeave: props.onMouseLeave,
      }),
    action((instance, newValue) => {
      instance.start = newValue.start;
      instance.end = newValue.end;
      instance.color = newValue.color;
      instance.borderColor = newValue.borderColor;
      instance.hideFlags = newValue.hideFlags;
      instance.hideBody = newValue.hideBody;
      instance.onMouseDown = newValue.onMouseDown;
      instance.onMouseUp = newValue.onMouseUp;
      instance.onMouseMove = newValue.onMouseMove;
      instance.onMouseEnter = newValue.onMouseEnter;
      instance.onMouseLeave = newValue.onMouseLeave;
    }),
    props,
  );

  useEffect(() => {
    return model.addHighlightRange(highlightRangeModel);
  }, [model, highlightRangeModel]);

  return null;
});
