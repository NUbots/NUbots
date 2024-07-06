import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observer } from "mobx-react";

import { Button } from "../../button/button";
import { lightAndDarkDecorator } from "../../storybook/color_mode";
import { FileDialogController } from "../controller";
import { FileSaveDialog } from "../save_dialog";

import { createFakeFileDialogModel, sampleFileSystem } from "./sample_file_system";

const meta: Meta<typeof FileSaveDialog> = {
  id: "components/FileDialog/Save",
  title: "components/FileDialog",
  component: FileSaveDialog,
  decorators: [lightAndDarkDecorator({ className: "w-full", horizontal: true })],
};

export default meta;

export const Default: StoryObj<typeof FileSaveDialog> = {
  name: "File Save Dialog",
  render: () => {
    // Create a sample file system to pick from
    const fileSystem = sampleFileSystem();

    // Keeps track of files selected
    const [selectedFile, setSelectedFile] = React.useState<string | undefined>();

    // Model and controller for the dialog component
    const model = createFakeFileDialogModel();
    const controller = FileDialogController.of({ model });

    // Update the list of entries when the dialog path changes
    const onDialogPathChange = (newPath: string) => {
      // Trim trailing slash
      const newPathTrimmed = newPath.endsWith("/") && newPath !== "/" ? newPath.slice(0, -1) : newPath;

      // Get entries for the path
      const entries = fileSystem[newPathTrimmed];

      // Set the entries for the new path
      controller.setCurrentPathEntries(entries);

      // Set an error if the path wasn't found. Note this is not the same as setting an empty list of
      // entries for paths that exist, but have no children.
      controller.setError(entries ? "" : `Path not found: ${newPath}`);
    };

    // Set our list of selected files when the dialog closes
    const onDialogClose = (selectedFile: string) => {
      setSelectedFile(selectedFile);
    };

    const Story = observer(() => (
      <div className="p-4">
        <Button onClick={controller.toggle}>Save File</Button>

        {model.isShown ? (
          <FileSaveDialog
            title="Pick a file to save to"
            model={model}
            controller={controller}
            onPathChange={onDialogPathChange}
            onClose={onDialogClose}
          />
        ) : null}

        {selectedFile ? (
          <>
            <div className="font-bold mt-3">Chosen file:</div>
            <div className="mt-3">{selectedFile}</div>
          </>
        ) : (
          <div className="mt-3">No file chosen.</div>
        )}
      </div>
    ));

    return <Story />;
  },
};
