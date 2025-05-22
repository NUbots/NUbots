import React, { useCallback } from "react";
import { observer } from "mobx-react";

import { useRpcController } from "../../../hooks/use_rpc_controller";
import { IconButton } from "../../icon_button/view";
import { NbsScrubberModel } from "../model";

import { NbsScrubberController } from "./controller";

export interface NbsScrubberViewProps {
  model: NbsScrubberModel;
  onFocus: () => void;
}

export const NbsScrubberView = observer(({ model, onFocus }: NbsScrubberViewProps) => {
  const buttonDivider = <span className="inline-block h-6 border-r border-divider mx-2"></span>;
  const controller = useRpcController((network) => new NbsScrubberController(model, network), [model]);

  return (
    <div className="flex justify-between items-center gap-1 h-12 px-2" onFocus={onFocus}>
      <NbsScrubberLeftControls model={model} controller={controller} />
      {buttonDivider}
      <NbsScrubberRangeSlider model={model} controller={controller} />
      {buttonDivider}
      <NbsScrubberRightControls model={model} controller={controller}>
        <IconButton title="Close scrubber" onClick={controller.close}>
          close
        </IconButton>
      </NbsScrubberRightControls>
    </div>
  );
});

export interface NbsScrubberControlsOpts {
  model?: NbsScrubberModel;
  controller?: NbsScrubberController;
  children?: React.ReactNode;
}

export const NbsScrubberLeftControls = observer(({ model, controller, children }: NbsScrubberControlsOpts) => {
  return (
    <>
      <IconButton
        title={model?.playbackState === "playing" ? "Pause" : "Play"}
        onClick={controller?.togglePlayback}
        disabled={!model}
      >
        {model?.playbackState === "playing" ? "pause" : "play_arrow"}
      </IconButton>
      <IconButton onClick={controller?.playSlower} title="Play slower" disabled={!model}>
        fast_rewind
      </IconButton>
      <IconButton onClick={controller?.playFaster} title="Play faster" disabled={!model}>
        fast_forward
      </IconButton>
      <span>
        {!model ? (
          "1x"
        ) : model.playbackSpeed >= 0 ? (
          model.playbackSpeedModifier + "x"
        ) : (
          <>
            {/* Shows playback speeds less than 1 as a fraction */}
            <sup>1</sup>&frasl;<sub>{Math.pow(2, Math.abs(model.playbackSpeed))}</sub>x
          </>
        )}
      </span>
      {children}
    </>
  );
});

export const NbsScrubberRightControls = observer(({ model, controller, children }: NbsScrubberControlsOpts) => {
  return (
    <>
      <IconButton
        color={model?.playbackRepeat ? "primary" : "default"}
        onClick={controller?.toggleRepeat}
        title={model?.playbackRepeat ? "Turn repeat off" : "Turn repeat on"}
        disabled={!model}
      >
        repeat
      </IconButton>
      {children}
    </>
  );
});

const NbsScrubberRangeSlider = observer(
  ({ model, controller }: { model: NbsScrubberModel; controller: NbsScrubberController }) => {
    const onInput = useCallback(
      (event: React.FormEvent<HTMLInputElement>) => controller.seekToPercent(Number(event.currentTarget.value)),
      [model, controller],
    );

    return (
      <div className="flex flex-col flex-grow">
        <div className="flex justify-between">
          <span title={`${model.startFormatted}―${model.endFormatted}`}>{model.name}</span>
          <NbsScrubberTimestamp model={model} />
        </div>
        <NbsScrubberInput model={model} onInput={onInput} />
      </div>
    );
  },
);

// This is a separate component because it re-renders a lot during playback
const NbsScrubberTimestamp = observer(({ model }: { model: NbsScrubberModel }) => {
  return <span>{model.currentFormatted}</span>;
});

// This is a separate component because it re-renders a lot during playback
const NbsScrubberInput = observer(
  ({ model, onInput }: { model: NbsScrubberModel; onInput: (event: React.FormEvent<HTMLInputElement>) => void }) => {
    return (
      <input
        className="w-full"
        type="range"
        onInput={onInput}
        value={model.percentPlayed}
        min={0}
        step={0.001}
        max={1}
      />
    );
  },
);
