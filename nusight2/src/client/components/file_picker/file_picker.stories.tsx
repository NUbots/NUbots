import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observer } from "mobx-react";

import { FilePickerController } from "./controller";
import { FilePickerEntry, FilePickerModel, parsePathSegments } from "./model";
import { FilePicker } from "./view";

const meta: Meta<typeof FilePicker> = {
  title: "components/FilePicker",
  component: FilePicker,
  parameters: {
    layout: "fullscreen",
  },
};

export default meta;

type Story = StoryObj<typeof FilePicker>;

export const Default: Story = {
  name: "FilePicker",
  render: () => {
    // Create a sample file system to pick from
    const fileSystem = sampleFileSystem();

    // Keeps track of files picked
    const [pickedFiles, setPickedFiles] = React.useState<string[]>([]);

    // Create the file picker model
    const model = FilePickerModel.of({
      currentPath: "/",
      currentPathEntries: undefined,
      recentPaths: JSON.parse(localStorage.getItem("nusight:recently_picked_paths") ?? "[]"),
      quickPaths: [
        {
          name: "/",
          path: "/",
        },
        {
          name: "nusight2",
          path: "/nusight2",
        },
        {
          name: "NotFound",
          path: "/path/that/does/not/exist",
        },
      ],
    });

    // Create the file picker controller
    const controller = FilePickerController.of({ model });

    // Update the list of entries when the picker path changes
    const onPickerPathChange = (newPath: string) => {
      // Trim trailing slash
      const newPathTrimmed = newPath.endsWith("/") && newPath !== "/" ? newPath.slice(0, -1) : newPath;

      // Get entries for the path
      const entries = fileSystem[newPathTrimmed];

      // Set the entries for the new path
      controller.setCurrentPathEntries(entries);

      // Set an error if the path wasn't found. Note this is not the same as
      controller.setError(entries ? "" : `Path not found: ${newPath}`);
    };

    // Set our list of picked files when the picker closes
    const onPickerClose = (pickedFiles: string[]) => {
      setPickedFiles(pickedFiles);
    };

    const Story = observer(() => (
      <div className="p-4">
        <button className="border rounded px-3 py-1 hover:bg-gray-100 active:bg-gray-300" onClick={controller.toggle}>
          Open file picker
        </button>

        {model.isShown ? (
          <FilePicker
            title="Pick a file"
            model={model}
            controller={controller}
            onPathChange={onPickerPathChange}
            onClose={onPickerClose}
          />
        ) : null}

        {pickedFiles.length > 0 ? (
          <>
            <div className="font-bold mt-3">Picked files:</div>
            <ul className="list-disc">
              {pickedFiles.map((file) => (
                <li key={file} className="pl-2 list-inside">
                  {file}
                </li>
              ))}
            </ul>
          </>
        ) : (
          <div className="mt-3">No file picked.</div>
        )}
      </div>
    ));

    return <Story />;
  },
};

function sampleFileSystem(): { [key: string]: FilePickerEntry[] } {
  return {
    "/": [
      sampleEntry("directory", "/nusight2"),
      sampleEntry("directory", "/recordings"),
      sampleEntry("directory", "/empty"),
    ],
    "/nusight2": [sampleEntry("directory", "/nusight2/src")],
    "/nusight2/src": [sampleEntry("directory", "/nusight2/src/client")],
    "/nusight2/src/client": [sampleEntry("directory", "/nusight2/src/client/components")],
    "/nusight2/src/client/components": [
      sampleEntry("directory", "/nusight2/src/client/components/file_tree"),
      sampleEntry("directory", "/nusight2/src/client/components/file_picker"),
      sampleEntry("directory", "/nusight2/src/client/components/nbs_scrubber"),
      sampleEntry("file", "/nusight2/src/client/components/common.tsx"),
      sampleEntry("file", "/nusight2/src/client/components/common.css"),
    ],
    "/nusight2/src/client/components/file_tree": [],
    "/nusight2/src/client/components/file_picker": [
      sampleEntry("file", "/nusight2/src/client/components/file_picker/file_picker.stories.tsx"),
      sampleEntry("file", "/nusight2/src/client/components/file_picker/view.tsx"),
    ],
    "/nusight2/src/client/components/nbs_scrubber": [
      sampleEntry("file", "/nusight2/src/client/components/nbs_scrubber/controller.ts"),
      sampleEntry("file", "/nusight2/src/client/components/nbs_scrubber/model.ts"),
      sampleEntry("file", "/nusight2/src/client/components/nbs_scrubber/network.ts"),
      sampleEntry("file", "/nusight2/src/client/components/nbs_scrubber/view.tsx"),
    ],
    "/recordings": [
      sampleEntry("file", "/recordings/a.nbs"),
      sampleEntry("file", "/recordings/b.nbs"),
      sampleEntry("file", "/recordings/c.nbs"),
      sampleEntry("file", "/recordings/d.nbs"),
      sampleEntry("file", "/recordings/e.nbs"),
      sampleEntry("file", "/recordings/f.nbs"),
    ],
    "/empty": [],
  };
}

function sampleEntry(type: "file" | "directory", path: string): FilePickerEntry {
  const segments = parsePathSegments(path);

  // Random size between 10B and 3GB
  const size = Math.floor(Math.random() * 3000000000) + 10;

  // Random date this year
  const dateModified = new Date(
    new Date().getFullYear(),
    Math.floor(Math.random() * 12),
    Math.floor(Math.random() * 28),
    Math.floor(Math.random() * 24),
    Math.floor(Math.random() * 60),
    Math.floor(Math.random() * 60),
  );

  return {
    type,
    name: segments[segments.length - 1].name,
    path,
    size: type === "file" ? size : 0,
    dateModified,
  };
}
