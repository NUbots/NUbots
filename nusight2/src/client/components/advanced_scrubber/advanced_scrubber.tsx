import React, { useCallback, useMemo } from "react";
import { NbsScrubberController } from "@components/nbs_scrubbers/nbs_scrubber/controller";
import { NbsScrubberLeftControls, NbsScrubberRightControls } from "@components/nbs_scrubbers/nbs_scrubber/view";
import { TimelineCameraView } from "@components/timeline/model";
import { useRpcController } from "@hooks/use_rpc_controller";
import { observable } from "mobx";
import { observer } from "mobx-react";

import { AdvancedScrubberScrubberModel } from "./model";
import { PacketTimeline } from "./packet_timeline/packet_timeline";

/**
 * Inline scrubber position slider.
 *
 * A plain range input that seeks by percentage. A minimap/overview zoom feature
 * could be layered on top in the future.
 */
const ScrubberSlider = observer(function ScrubberSlider(props: {
  model: AdvancedScrubberScrubberModel;
  controller?: NbsScrubberController;
}) {
  const { model, controller } = props;
  const { scrubber } = model;

  const onInput = useCallback(
    (event: React.FormEvent<HTMLInputElement>) => {
      controller?.seekToPercent(Number(event.currentTarget.value));
    },
    [controller],
  );

  return (
    <div className="flex-1 flex flex-col gap-1 min-w-0">
      <div className="flex justify-between text-xs text-auto-secondary px-0.5">
        <span>{scrubber.currentFormatted}</span>
      </div>
      <input
        className="w-full"
        type="range"
        onInput={onInput}
        value={scrubber.percentPlayed}
        min={0}
        step={0.001}
        max={1}
        disabled={!controller}
      />
    </div>
  );
});

export const AdvancedScrubber = observer(function AdvancedScrubber(props: { model: AdvancedScrubberScrubberModel }) {
  const { scrubber, indices = [] } = props.model;
  const divider = <div className="h-2/3 border-l border-auto" />;

  const timelineCameraView = useMemo<TimelineCameraView>(
    () =>
      observable({
        centre: (scrubber.start + scrubber.end) / 2n,
        // Widen the initial view slightly to ensure packets at the start and end are visible
        range: scrubber.end - scrubber.start + (scrubber.end - scrubber.start) / 20n,
      }),
    [scrubber],
  );

  const controller = useRpcController((network) => new NbsScrubberController(scrubber, network), [scrubber]);

  return (
    <div className="w-full h-full bg-auto-surface-0 text-auto-primary flex flex-col justify-start items-center">
      <div className="w-full flex justify-between items-center px-4 py-2 bg-auto-surface-2">
        <span className="font-medium text-base">{scrubber.name}</span>
        <div className="flex h-full items-center gap-2 text-sm">
          <span className="text-secondary">Start</span>
          <span>{scrubber.startFormatted}</span>
          {divider}
          <span className="text-secondary">End</span>
          <span>{scrubber.endFormatted}</span>
          {divider}
          <span className="text-secondary">Duration</span>
          <span>{scrubber.durationFormatted}</span>
        </div>
      </div>
      <div className="w-full flex items-center gap-4 px-4 py-1 border-y border-auto">
        <div className="flex items-center gap-1">
          <NbsScrubberLeftControls model={scrubber} controller={controller} />
        </div>
        {divider}
        {/* Plain scrubber slider; a minimap/overview zoom feature is not implemented in this view. */}
        <ScrubberSlider model={props.model} controller={controller} />
        {divider}
        <NbsScrubberRightControls model={scrubber} controller={controller} />
      </div>
      <PacketTimeline
        scrubber={scrubber}
        indices={indices}
        controller={controller}
        timelineCameraView={timelineCameraView}
      />
    </div>
  );
});
