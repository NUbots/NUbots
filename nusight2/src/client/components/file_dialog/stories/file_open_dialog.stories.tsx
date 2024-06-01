import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observer } from "mobx-react";

import { Button } from "../../button/button";
import { lightAndDarkDecorator } from "../../storybook/color_mode";
import { FileDialogController } from "../controller";
import { FileOpenDialog } from "../open_dialog";
import { createFakeFileDialogModel, sampleFileSystem } from "../stories/sample_file_system";

const meta: Meta<typeof FileOpenDialog> = {
  id: "components/FileDialog/Open",
  title: "components/FileDialog",
  component: FileOpenDialog,
  decorators: [lightAndDarkDecorator({ className: "w-full", horizontal: true })],
};

export default meta;

export const Default: StoryObj<typeof FileOpenDialog> = {
  name: "File Open Dialog",
  render: () => {
    // Create a sample file system to pick from
    const fileSystem = sampleFileSystem();

    // Keeps track of files selected
    const [selectedFiles, setSelectedFiles] = React.useState<string[]>([]);

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
    const onDialogClose = (selectedFiles: string[]) => {
      setSelectedFiles(selectedFiles);
    };

    const Story = observer(() => (
      <div className="p-4">
        <Button onClick={controller.toggle}>Open File</Button>

        {model.isShown ? (
          <FileOpenDialog
            title="Pick a file"
            model={model}
            controller={controller}
            type="file"
            onPathChange={onDialogPathChange}
            onClose={onDialogClose}
          />
        ) : null}

        {selectedFiles.length > 0 ? (
          <>
            <div className="font-bold mt-3">Chosen files:</div>
            <ul className="list-disc">
              {selectedFiles.map((file) => (
                <li key={file} className="pl-2 list-inside">
                  {file}
                </li>
              ))}
            </ul>
          </>
        ) : (
          <div className="mt-3">No file(s) chosen.</div>
        )}
      </div>
    ));

    return <Story />;
  },
};
