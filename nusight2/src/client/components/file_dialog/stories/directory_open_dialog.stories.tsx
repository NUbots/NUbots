import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observer } from "mobx-react";

import { Button } from "../../button/button";
import { lightAndDarkDecorator } from "../../storybook/color_mode";
import { FileDialogController } from "../controller";
import { FileOpenDialog } from "../open_dialog";
import { createFakeFileDialogModel, sampleFileSystem } from "../stories/sample_file_system";

const meta: Meta<typeof FileOpenDialog> = {
  id: "components/FileDialog/Directory",
  title: "components/FileDialog",
  component: FileOpenDialog,
  decorators: [lightAndDarkDecorator({ className: "w-full", horizontal: true })],
};

export default meta;

export const Default: StoryObj<typeof FileOpenDialog> = {
  name: "Directory Open Dialog",
  render: () => {
    // Create a sample file system to pick from
    const fileSystem = sampleFileSystem();

    // Keeps track of directory selected
    const [selectedDirectories, setSelectedDirectories] = React.useState<string[]>([]);

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

    // Set our list of selected directories when the dialog closes
    const onDialogClose = (selectedDirectories: string[]) => {
      setSelectedDirectories(selectedDirectories);
    };

    const Story = observer(() => (
      <div className="p-4">
        <Button onClick={controller.toggle}>Open Directory</Button>

        {model.isShown ? (
          <FileOpenDialog
            title="Pick a directory"
            model={model}
            controller={controller}
            type="directory"
            onPathChange={onDialogPathChange}
            onClose={onDialogClose}
          />
        ) : null}

        {selectedDirectories.length > 0 ? (
          <div className="font-bold mt-3">Chosen Directory: {selectedDirectories[0]}</div>
        ) : (
          <div className="mt-3">No directory chosen.</div>
        )}
      </div>
    ));

    return <Story />;
  },
};
