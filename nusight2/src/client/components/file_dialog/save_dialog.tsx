import React, { useCallback, useMemo, useRef, useState } from "react";
import { useEffect } from "react";
import { autorun, reaction } from "mobx";
import { observer } from "mobx-react";

import { PathSegments } from "../../file/path_segments";
import { Button } from "../button/button";

import { FileDialogController } from "./controller";
import { FileDialogModel } from "./model";
import { FileDialog, FileDialogContext, FileDialogProps } from "./view";

export type FileSaveDialogProps = {
  /** Title of the dialog box */
  title: string;
  /** Dialog model to display data from */
  model: FileDialogModel;
  /** File dialog controller */
  controller: FileDialogController;
  /** Extension of the returned path */
  extension?: string;
  /** Callback for when the current path to display changes */
  onPathChange(newPath: string): void;
  /** Callback for when the dialog is closed or submitted */
  onClose(selectedFile?: string): void;
};

/**
 * Display a modal dialog for picking a file to save to.
 * This may either be an existing file or a new file path.
 */
export const FileSaveDialog = observer((props: FileSaveDialogProps) => {
  const { title, model, extension, controller, onClose, onPathChange } = props;

  const inputRef = useRef<HTMLInputElement>(null);

  // The value of the file name text input
  const [fileNameInput, setFileNameInput] = useState<string>("");

  // Notify our parent when the current path changes
  useEffect(() => {
    return autorun(() => {
      onPathChange(model.currentPath);
    });
  }, []);

  // Update file name input when a file is selected
  useEffect(() => {
    return reaction(
      () => model.lastSelectedEntryIndex,
      (idx) => {
        if (idx === undefined) {
          return;
        }
        setFileNameInput(model.currentPathEntriesSorted![idx].name);
      },
    );
  }, []);

  // Append the current extension to the given string. If the string is empty it is not appended.
  const appendExtension = useCallback(
    (path: string) => {
      const needsExtension = path.length > 0 && extension && !path.endsWith(extension);
      return needsExtension ? path + extension : path;
    },
    [extension],
  );

  // Highlight the file name without the extension
  function highlightFileName() {
    if (extension && fileNameInput.endsWith(extension)) {
      inputRef.current?.setSelectionRange(0, fileNameInput.length - extension.length);
    } else {
      inputRef.current?.select();
    }
  }

  // Deselect last selected file before setting the current file name
  function onFileNameInput(name: string) {
    controller.setLastSelectedEntryIndex(undefined);
    setFileNameInput(name);
  }

  // Save the directory the file was selected from and close the dialog
  function closeDialogWithSelectedFile() {
    const fileName = appendExtension(fileNameInput);
    const enteredPath = new PathSegments(model.currentPath).join(fileName);

    // Add the directory of the file to the recently selected paths
    const directory = enteredPath.slice(0, -1);
    controller.addRecentPaths([{ name: directory.last, path: directory.path }]);

    // Hide popup
    controller.toggle();

    // Notify our parent with the selected file
    onClose(enteredPath.path);
  }

  // The current save dialog only supports file pickers
  const type: FileDialogProps["type"] = "file";

  // Disable multi-file select for choosing a save file
  const fileDialogContext = useMemo(() => ({ allowMultiSelect: false, type }), []);

  return (
    /* File dialog with text input for selected file and button to open it */
    <FileDialogContext.Provider value={fileDialogContext}>
      <FileDialog
        title={title}
        model={model}
        controller={controller}
        type={type}
        onClose={(files) => onClose(files[0])}
      >
        <div className="text-base font-semibold leading-none w-30 whitespace-nowrap">
          File Name: {extension ? `(${extension})` : null}
        </div>
        <input
          ref={inputRef}
          autoFocus
          type="text"
          spellCheck={false}
          className="bg-auto-contrast-1 text-auto-primary hover:bg-auto-contrast-2 focus:bg-auto-surface-1 cursor-text rounded h-9 px-4 mx-4 w-full outline-none border-2 border-transparent focus:border-nusight-500"
          value={fileNameInput}
          onChange={(event) => {
            controller.setSelectedEntries([]);
            onFileNameInput(event.target.value);
          }}
          onKeyDown={(event) => {
            if (fileNameInput && event.key === "Enter") {
              closeDialogWithSelectedFile();
            }
          }}
          onFocus={highlightFileName}
          onBlur={() => setFileNameInput(appendExtension(fileNameInput))}
        />
        <Button color="primary" disabled={fileNameInput.trim().length === 0} onClick={closeDialogWithSelectedFile}>
          Save
        </Button>
      </FileDialog>
    </FileDialogContext.Provider>
  );
});
