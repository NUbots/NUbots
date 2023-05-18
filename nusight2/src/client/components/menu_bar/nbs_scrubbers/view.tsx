import React, { useEffect } from "react";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import { NUsightNetwork } from "../../../network/nusight_network";
import { Collapsible } from "../../collapsible/view";
import { FilePicker } from "../../file_picker/view";

import { NbsScrubbersController } from "./controller";
import { IconInfo, IconNbs, IconNbsFilled } from "./icons";
import { NbsScrubbersModel } from "./model";
import { NbsScrubberView } from "./nbs_scrubber/view";
import { NbsScrubbersNetwork } from "./network";

export function withNbsScrubbers(nusightNetwork: NUsightNetwork) {
  const model = NbsScrubbersModel.of();
  const network = NbsScrubbersNetwork.of(nusightNetwork, model);
  const controller = NbsScrubbersController.of({ network, model });

  const showFilePicker = action(() => {
    model.filePicker.isShown = true;
  });

  const toggleScrubbers = action(() => {
    model.showScrubbers = !model.showScrubbers;
  });

  const scrubbers = observer(() => (
    <Collapsible open={model.showScrubbers} onToggle={toggleScrubbers} className="!p-0 !border-0">
      <NbsScrubbers model={model} controller={controller} />
    </Collapsible>
  ));

  const scrubbersToggle = observer(() => (
    <>
      <NbsScrubbersToggle showScrubbers={model.showScrubbers} onClick={showFilePicker} onToggle={toggleScrubbers} />
      {model.filePicker.isShown ? <NbsFilePicker model={model} controller={controller} /> : null}
    </>
  ));

  return { NbsScrubbers: scrubbers, NbsScrubbersToggle: scrubbersToggle };
}

interface NbsScrubbersProps {
  model: NbsScrubbersModel;
  controller: NbsScrubbersController;
}

const NbsScrubbers = observer(({ model, controller }: NbsScrubbersProps) => {
  // Toggle playback of the last scrubber interacted with when the space bar is pressed
  useEffect(() => {
    function handleKeyPress(event: any) {
      if (event.key !== " ") {
        return;
      }

      const activeElement = document.activeElement;

      if (activeElement) {
        const tagName = activeElement.tagName.toLowerCase();

        const elementsThatRespondToSpaceBar = ["select", "button", "textarea"];
        if (elementsThatRespondToSpaceBar.includes(tagName)) {
          return;
        }

        // Allow the space bar to be used for toggling on range inputs
        if (tagName === "input" && activeElement.getAttribute("type") !== "range") {
          return;
        }
      }

      controller.togglePlaybackForLastActiveScrubber();
    }

    document.addEventListener("keypress", handleKeyPress);

    return () => {
      document.removeEventListener("keypress", handleKeyPress);
    };
  }, []);

  return (
    <div className="divide-y divide-gray-50 border-y border-gray-300">
      {model.scrubberList.map((scrubber) => (
        <NbsScrubberView key={scrubber.id} model={scrubber} controller={controller} />
      ))}
      {model.scrubberList.length === 0 ? (
        <div className="flex px-2 h-12 items-center bg-gray-200">
          <IconInfo className="text-icon w-6 h-6 mr-2" />
          <span>No scrubbers currently loaded. Click the “Scrub NBS” button to load NBS files for scrubbing.</span>
        </div>
      ) : null}
    </div>
  );
});

interface NbsScrubbersToggleProps {
  showScrubbers: boolean;
  onClick: () => void;
  onToggle: () => void;
}

function NbsScrubbersToggle({ showScrubbers, onClick, onToggle }: NbsScrubbersToggleProps) {
  // The arbitrary class values here are to make the nbs toggle button line up with and look like
  // the robot select button it's next to. TODO: extract something like a common `<MenuBarButton>`
  // component that can be used for both.
  return (
    <div className="flex relative">
      <button
        className={classNames(
          "inline-flex flex-col items-center justify-center bg-transparent px-3 h-[60px] hover:bg-[#f8f8f8] focus:bg-[#f8f8f8] active:bg-[#eee]",
        )}
        onClick={onClick}
      >
        {showScrubbers ? (
          <IconNbsFilled className="w-7 h-7 mt-1 -ml-[20px]" />
        ) : (
          <IconNbs className="w-7 h-7 mt-1 -ml-[20px]" />
        )}
        <span className="text-[0.7rem]">Scrub NBS</span>
      </button>
      <button className="absolute right-0 top-0 mt-1.5 mr-2.5 px-0.5 hover:bg-gray-200 h-8 rounded" onClick={onToggle}>
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className={classNames("w-5 h-5 transform rotate-0 transition-transform", { "-rotate-180": showScrubbers })}
        >
          <polyline points="6 9 12 15 18 9"></polyline>
        </svg>
      </button>
    </div>
  );
}

interface NbsFilePickerProps {
  model: NbsScrubbersModel;
  controller: NbsScrubbersController;
}

const NbsFilePicker = observer(({ model, controller }: NbsFilePickerProps) => {
  return (
    <FilePicker
      title="Pick one or more NBS files to load"
      model={model.filePicker}
      controller={controller.filePickerController}
      onPathChange={(newPath) => controller.listFiles(newPath)}
      onClose={(pickedFiles: string[]) => {
        if (pickedFiles?.length > 0) {
          controller.load(pickedFiles);
        }
      }}
    />
  );
});
