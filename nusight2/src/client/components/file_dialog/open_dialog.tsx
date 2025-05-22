import React, { useMemo } from "react";
import { useEffect } from "react";
import { autorun } from "mobx";
import { observer } from "mobx-react";

import { PathSegments } from "../../file/path_segments";
import { Button } from "../button/button";
import { Icon } from "../icon/view";

import { FileDialogController } from "./controller";
import { FileDialogModel } from "./model";
import { FileDialog, FileDialogContext } from "./view";

export type FileOpenDialogProps = {
  title: string;
  model: FileDialogModel;
  controller: FileDialogController;
  type: "file" | "directory";
  onPathChange(newPath: string): void;
  onClose(selectedFiles: string[]): void;
};

/** Displays a modal dialog for picking files */
export const FileOpenDialog = observer((props: FileOpenDialogProps) => {
  const { title, model, controller, type, onClose, onPathChange } = props;

  // Listen for a change in the current path
  useEffect(() => {
    return autorun(() => {
      const newPath = model.currentPath;

      // If this is a directory dialog, select the new path we just navigated to
      if (type === "directory") {
        controller.setSelectedEntries([
          {
            type: "directory",
            name: new PathSegments(newPath).last,
            path: newPath,
            size: 0,
            dateModified: new Date(),
          },
        ]);
      }
      // Notify our parent that the path changed
      onPathChange(newPath);
    });
  }, []);

  // Save the parent directories the files/directories were selected from and close the dialog
  function closeDialogWithSelectedPaths() {
    // Update recent paths
    controller.updateRecentPathsFromSelection();

    // Get a copy of the selected files or directories
    const selectedPaths = type === "file" ? model.selectedFilePaths.slice() : model.selectedDirectoryPaths.slice();

    // Close the dialog
    controller.toggle();

    // Notify our parent with the list of selected paths
    onClose(selectedPaths);
  }

  const fileDialogContext = useMemo(() => ({ allowMultiSelect: type === "file", type }), [type]);

  return (
    /* File dialog with display for selected files or directories to open them */
    <FileDialogContext.Provider value={fileDialogContext}>
      <FileDialog title={title} model={model} controller={controller} type={type} onClose={onClose}>
        {type === "directory" ? (
          <FileOpenDialogSummary
            paths={model.selectedDirectoryPaths}
            summary={model.selectedDirectoriesSummary}
            onClear={() => controller.setSelectedEntries([])}
          />
        ) : (
          <FileOpenDialogSummary
            paths={model.selectedFilePaths}
            summary={model.selectedFilesSummary}
            onClear={() => controller.setSelectedEntries([])}
          />
        )}

        <Button
          color="primary"
          disabled={
            type === "directory" ? model.selectedDirectoryPaths.length === 0 : model.selectedFilePaths.length === 0
          }
          onClick={closeDialogWithSelectedPaths}
        >
          Open
        </Button>
      </FileDialog>
    </FileDialogContext.Provider>
  );
});

function FileOpenDialogSummary(props: { paths: ReadonlyArray<string>; summary: string; onClear: () => void }) {
  return props.paths.length > 0 ? (
    <div className="flex items-center flex-grow">
      {props.summary}
      <button
        className="ml-2 p-1 inline-block rounded-full hover:bg-auto-contrast-2 text-auto-secondary hover:text-auto-primary leading-none"
        title="Clear selection"
        onClick={props.onClear}
      >
        <Icon className="block">close</Icon>
      </button>
    </div>
  ) : null;
}
