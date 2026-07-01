import React, { createContext, useMemo } from "react";
import { useUpdatable } from "@hooks/use_updatable";

import { TimelineController, TimelineEventHandler } from "./controller";
import { TimelineCameraView, TimelineModel } from "./model";

interface TimelineContext {
  model: TimelineModel;
  controller: TimelineController;
}

export const timelineContext = createContext<TimelineContext | undefined>(undefined);

export interface TimelineViewProps {
  view: TimelineCameraView;
  className?: string;
  onViewRangeChange?: TimelineEventHandler;
  onViewCentreChange?: TimelineEventHandler;
  children?: React.ReactNode;
}

export function TimelineView({ view, className, onViewRangeChange, onViewCentreChange, children }: TimelineViewProps) {
  // Create a single model instance that updates when props change
  const model = useUpdatable(
    () => new TimelineModel(view),
    (instance, [view]) => (instance.view = view),
    [view],
  );

  const controller = useMemo(
    () => new TimelineController(model, { onViewRangeChange, onViewCentreChange }),
    [model, onViewRangeChange, onViewRangeChange],
  );

  // Provide model to descendent timeline components
  const context = useMemo<TimelineContext>(() => ({ model, controller }), [model, controller]);

  return (
    <timelineContext.Provider value={context}>
      <div className={className}>{children}</div>
    </timelineContext.Provider>
  );
}
