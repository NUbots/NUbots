import React from "react";
import classNames from "classnames";
import { observer } from "mobx-react";

import { NbsScrubbersController } from "../controller";

import { IconClose } from "./icons";
import { IconPause } from "./icons";
import { IconPlay } from "./icons";
import { IconFastForward } from "./icons";
import { IconRewind } from "./icons";
import { IconRepeat } from "./icons";
import { NbsScrubberModel } from "./model";

export interface NbsScrubberViewProps {
  model: NbsScrubberModel;
  controller: NbsScrubbersController;
}

export const NbsScrubberView = observer(({ model, controller }: NbsScrubberViewProps) => {
  const buttonDivider = <span className="inline-block h-6 border-r border-divider mx-2"></span>;

  return (
    <div className="flex justify-between items-center gap-1 h-12 bg-gray-300 px-2">
      <NbsScrubberLeftControls model={model} controller={controller} />
      {buttonDivider}
      <NbsScrubberRangeSlider model={model} controller={controller} />
      {buttonDivider}
      <NbsScrubberRightControls model={model} controller={controller} />
    </div>
  );
});

const NbsScrubberLeftControls = observer(
  ({ model, controller }: { model: NbsScrubberModel; controller: NbsScrubbersController }) => {
    return (
      <>
        <IconButton
          title={model.playbackState === "playing" ? "Pause" : "Play"}
          onClick={() => controller.togglePlayback(model)}
        >
          {model.playbackState === "playing" ? <IconPause /> : <IconPlay />}
        </IconButton>
        <IconButton onClick={() => controller.playSlower(model)} title="Play slower">
          <IconRewind />
        </IconButton>
        <IconButton onClick={() => controller.playFaster(model)} title="Play faster">
          <IconFastForward />
        </IconButton>
        <span>
          {model.playbackSpeed >= 0 ? (
            model.playbackSpeedModifier + "x"
          ) : (
            <>
              {/* Shows playback speeds less than 1 as a fraction */}
              <sup>1</sup>&frasl;<sub>{Math.pow(2, Math.abs(model.playbackSpeed))}</sub>x
            </>
          )}
        </span>
      </>
    );
  },
);

const NbsScrubberRightControls = observer(
  ({ model, controller }: { model: NbsScrubberModel; controller: NbsScrubbersController }) => {
    return (
      <>
        <IconButton
          toggled={model.playbackRepeat}
          onClick={() => controller.toggleRepeat(model)}
          title={model.playbackRepeat ? "Turn repeat off" : "Turn repeat on"}
        >
          <IconRepeat className={model.playbackRepeat ? "" : ""} />
        </IconButton>
        <IconButton title="Close scrubber" onClick={() => controller.close(model)}>
          <IconClose />
        </IconButton>
      </>
    );
  },
);

const NbsScrubberRangeSlider = observer(
  ({ model, controller }: { model: NbsScrubberModel; controller: NbsScrubbersController }) => {
    return (
      <div className="flex flex-col flex-grow">
        <div className="flex justify-between">
          <span title={`${model.startFormatted}―${model.endFormatted}`}>{model.name}</span>
          <span>{model.currentFormatted}</span>
        </div>
        <input
          className="w-full"
          type="range"
          onMouseDown={() => controller.startSeeking(model)}
          onMouseUp={() => controller.stopSeeking(model)}
          onInput={(event) => controller.seek(model, event)}
          value={model.percentPlayed}
          min={0}
          step={0.001}
          max={1}
        />
      </div>
    );
  },
);

interface IconButtonProps {
  onClick: () => void;
  className?: string;
  title?: string;
  toggled?: boolean;
  disabled?: boolean;
  children: React.ReactNode;
}

const IconButton = observer(({ onClick, className, title, toggled, disabled, children }: IconButtonProps) => (
  <button
    className={classNames(
      "inline-flex w-8 h-8 items-center justify-center rounded hover:bg-white focus:bg-white active:bg-gray-200",
      {
        "bg-white text-nusight-500": toggled,
        "bg-gray-100 text-icon": !toggled,
      },
      className,
    )}
    onClick={onClick}
    title={title}
    disabled={disabled}
  >
    {children}
  </button>
));
