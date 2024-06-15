import React, { useMemo } from "react";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import { useRpcController } from "../../hooks/use_rpc_controller";
import { AppModel } from "../app/model";
import { Collapsible } from "../collapsible/view";
import { FileDialogModel } from "../file_dialog/model";
import { useFileDialogNetwork } from "../file_dialog/network";
import { FileOpenDialog } from "../file_dialog/open_dialog";
import { Icon } from "../icon/view";
import { IconNbs, IconNbsFilled } from "../icons";

import { NbsScrubberDialogController } from "./controller";
import { NbsScrubberModel } from "./model";
import { NbsScrubberView } from "./nbs_scrubber/view";
import { NbsScrubbersViewModel } from "./view_model";

export function withNbsScrubbers(appModel: AppModel) {
  const viewModel = new NbsScrubbersViewModel(appModel.scrubbersModel);

  const setLastActiveScrubber = action((scrubber: NbsScrubberModel) => {
    appModel.scrubbersModel.lastActiveScrubber = scrubber;
  });

  const Scrubbers = observer(() => {
    return (
      <Collapsible open={viewModel.areScrubbersVisible} onToggle={viewModel.toggleScrubbers} className="!p-0 !border-0">
        <NbsScrubbers viewModel={viewModel} onScrubberFocused={setLastActiveScrubber} />
      </Collapsible>
    );
  });

  const ScrubbersToggle = observer(() => (
    <>
      <NbsScrubbersToggle
        showScrubbers={viewModel.areScrubbersVisible}
        onClick={viewModel.showFileDialog}
        onToggle={viewModel.toggleScrubbers}
      />
      {viewModel.isFileDialogVisible ? <ScrubberFileOpenDialog onClose={viewModel.hideFileDialog} /> : null}
    </>
  ));

  return { NbsScrubbers: Scrubbers, NbsScrubbersToggle: ScrubbersToggle };
}

interface NbsScrubbersProps {
  viewModel: NbsScrubbersViewModel;
  onScrubberFocused: (scrubber: NbsScrubberModel) => void;
}

const NbsScrubbers = observer(({ viewModel, onScrubberFocused }: NbsScrubbersProps) => {
  return (
    <div className="shadow">
      {viewModel.scrubberList.map((scrubber) => (
        <NbsScrubberView key={scrubber.id} model={scrubber} onFocus={() => onScrubberFocused(scrubber)} />
      ))}
      {viewModel.scrubberList.length === 0 ? (
        <div className="flex px-2 h-12 items-center bg-auto-surface-2">
          <Icon className="mr-2">info</Icon>
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
        className={classNames("inline-flex flex-col items-center justify-center bg-transparent px-3 h-[60px]")}
        onClick={onClick}
      >
        {showScrubbers ? (
          <IconNbsFilled className="w-7 h-7 mt-1 -ml-[20px] fill-blue-500 text-white" />
        ) : (
          <IconNbs className="w-7 h-7 mt-1 -ml-[20px]" />
        )}
        <span className="text-[0.7rem]">Scrub NBS</span>
      </button>
      <button
        className="absolute right-0 top-0 mt-1.5 mr-2.5 px-0.5 hover:bg-auto-contrast-1 h-8 rounded inline-flex items-center"
        onClick={onToggle}
      >
        <Icon size="20" weight="500" rotate={showScrubbers ? 180 : 0} className="transition-transform">
          expand_more
        </Icon>
      </button>
    </div>
  );
}

interface ScrubberFileOpenDialogProps {
  onClose: () => void;
}

const ScrubberFileOpenDialog = observer(({ onClose }: ScrubberFileOpenDialogProps) => {
  const model = useMemo(
    () =>
      FileDialogModel.of({
        currentPath: JSON.parse(localStorage.getItem("nusight:last_selected_path") ?? '"."'),
        recentPaths: JSON.parse(localStorage.getItem("nusight:recently_selected_paths") ?? "[]"),
      }),
    [],
  );

  const controller = useRpcController((network) => new NbsScrubberDialogController(network, model), []);
  const network = useFileDialogNetwork(model);

  return (
    <FileOpenDialog
      title="Pick one or more NBS files to load"
      model={model}
      controller={controller}
      onPathChange={(newPath) => network.listFiles({ directory: newPath, type: "nbs" })}
      type="file"
      onClose={(selectedFiles: string[]) => {
        controller.load(selectedFiles);
        onClose();
      }}
    />
  );
});
