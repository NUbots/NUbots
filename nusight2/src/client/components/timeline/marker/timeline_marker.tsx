import { useContext, useEffect } from "react";
import { usePropertyChange } from "@hooks/use_property_change";
import { action, observable } from "mobx";
import { observer } from "mobx-react";

import { TimelineMarkerModel } from "../model";
import { timelineContext } from "../view";

/** Type where the specified keys are optional */
type SetOptional<T, K extends keyof T> = Omit<T, K> & Partial<Pick<T, K>>;

export type TimelineMarkerProps = SetOptional<TimelineMarkerModel, "color" | "scale">;

export const TimelineMarker = observer(function TimelineMarker(props: TimelineMarkerProps) {
  const context = useContext(timelineContext);
  if (!context) {
    throw new Error("Cannot have a <TimelineMarker> component outside of a <TimelineView>");
  }

  const { model } = context;

  // Create observable marker model from props that updates when any prop changes
  const markerModel = usePropertyChange<TimelineMarkerProps, TimelineMarkerModel>(
    () =>
      observable({
        timestamp: props.timestamp,
        text: props.text,
        color: props.color ?? "#ffffff",
        scale: props.scale ?? 1,
        hideTail: props.hideTail,
        clampToView: props.clampToView,
        onMouseDown: props.onMouseDown,
        onMouseMove: props.onMouseMove,
        onMouseUp: props.onMouseUp,
        onMouseEnter: props.onMouseEnter,
        onMouseLeave: props.onMouseLeave,
      }),
    action((instance, newValue) => {
      instance.timestamp = newValue.timestamp;
      instance.text = newValue.text;
      instance.color = newValue.color ?? "#ffffff";
      instance.scale = newValue.scale ?? 1;
      instance.hideTail = newValue.hideTail;
      instance.clampToView = newValue.clampToView;
      instance.onMouseDown = newValue.onMouseDown;
      instance.onMouseUp = newValue.onMouseUp;
      instance.onMouseMove = newValue.onMouseMove;
      instance.onMouseEnter = newValue.onMouseEnter;
      instance.onMouseLeave = newValue.onMouseLeave;
    }),
    props,
  );

  // Add marker to timeline model while this component is mounted
  useEffect(() => {
    return model.addMarker(markerModel);
  }, [model, markerModel]);

  return null;
});
