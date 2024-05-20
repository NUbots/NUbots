import React, { useState } from "react";
import { useEffect } from "react";
import { autorun } from "mobx";
import { observer } from "mobx-react";

import { siUnit } from "../../base/si_unit";
import { Button } from "../button/view";
import { Modal } from "../modal/view";

import { FilePickerController } from "./controller";
import { IconClose } from "./icons";
import { IconArrowBackward } from "./icons";
import { IconArrowForward } from "./icons";
import { IconArrowUpward } from "./icons";
import { IconChevron } from "./icons";
import { IconError } from "./icons";
import { IconFile } from "./icons";
import { IconFolder } from "./icons";
import { FilePickerModel } from "./model";
import { FilePickerPath } from "./model";

export type FilePickerProps = {
  title: string;
  model: FilePickerModel;
  controller: FilePickerController;
  onPathChange: (newPath: string) => void;
  onClose(pickedFiles: string[]): void;
};

/**
 * Displays a modal dialog for picking files.
 */
export const FilePicker = observer((props: FilePickerProps) => {
  const { title, model, controller, onClose, onPathChange } = props;

  // Notify our parent when the current path changes
  useEffect(() => {
    return autorun(() => {
      onPathChange(model.currentPath);
    });
  }, []);

  // Save the directories the files were picked from and close the picker
  function closePickerWithSelectedFiles() {
    // Update recent paths
    controller.updateRecentPathsFromSelection();

    // Get a copy of the selected files
    const pickedFiles = model.selectedFiles.slice();

    // Close the picker
    controller.toggle();

    // Notify our parent with the list of picked files
    onClose(pickedFiles);
  }

  return (
    <Modal
      onClose={() => {
        controller.toggle();
        onClose([]);
      }}
    >
      <div className="file-picker max-w-[100vw] w-[900px] flex flex-col h-max text-left">
        {/* Title */}
        <div className="text-base font-semibold px-4 py-4 leading-none">{title}</div>

        {/* Navigation buttons and path view */}
        <div className="border-y border-gray-300 dark:border-gray-800 py-1 px-2 flex">
          <FilePickerNavButtons model={model} controller={controller} />
          <FilePickerPathView model={model} controller={controller} />
        </div>

        {/*
          Main content: sidebar and file list.
          165px is the height of the header + nav bar + footer. We subtract that from the viewport height to get
          the max height of the content area, to ensure we always have the picker header, navbar, and footer visible.
        */}
        <div className="flex flex-grow h-[32rem] max-h-[calc(100vh_-_165px)]">
          <div className="border-r border-gray-300 dark:border-gray-800 flex w-52 shrink-0 h-full overflow-y-auto">
            <FilePickerSidebar model={model} controller={controller} />
          </div>
          <div className="flex-grow h-full overflow-auto">
            <FilePickerList model={model} controller={controller} />
          </div>
        </div>

        {/* Footer: selection summary and Open + Cancel buttons */}
        <div className="border-t border-gray-300 dark:border-gray-800 flex items-center justify-end p-4 gap-2">
          {model.selectedFiles.length > 0 && (
            <div className="flex items-center flex-grow">
              {model.selectedFilesSummary}
              <button
                className="ml-2 p-1 inline-block rounded-full hover:bg-blue-600 hover:text-black leading-none"
                title="Clear selection"
                onClick={() => controller.setSelectedFiles([])}
              >
                <IconClose className="w-5 h-5" />
              </button>
            </div>
          )}
          <Button type="primary" disabled={model.selectedFiles.length === 0} onClick={closePickerWithSelectedFiles}>
            Open
          </Button>
          <Button
            onClick={() => {
              controller.toggle();
              onClose([]);
            }}
          >
            Cancel
          </Button>
        </div>
      </div>
    </Modal>
  );
});

type FilePickerNavButtonsProps = {
  model: FilePickerModel;
  controller: FilePickerController;
};

/**
 * Displays Back, Forward, and Up navigation buttons for the file picker, and handles navigation.
 */
const FilePickerNavButtons = observer((props: FilePickerNavButtonsProps) => {
  const { model, controller } = props;

  return (
    <>
      <FilePickerNavButton
        Icon={IconArrowBackward}
        disabled={model.navigationHistory.back.length === 0}
        onClick={controller.navigateBackward}
      />
      <FilePickerNavButton
        Icon={IconArrowForward}
        disabled={model.navigationHistory.forward.length === 0}
        onClick={controller.navigateForward}
      />
      <FilePickerNavButton
        Icon={IconArrowUpward}
        disabled={model.currentPathSegments.length < 2}
        onClick={controller.navigateUpward}
      />
    </>
  );
});

type FilePickerNavButtonProps = {
  Icon: (props: { className?: string }) => JSX.Element;
  disabled: boolean;
  onClick: () => void;
};

/**
 * Displays a single navigation button
 */
const FilePickerNavButton = observer((props: FilePickerNavButtonProps) => {
  const { Icon, disabled, onClick } = props;
  return (
    <button
      className={`px-2 py-1 rounded ${
        disabled
          ? "opacity-40"
          : "hover:bg-gray-250 dark:hover:bg-gray-700 focus-visible:bg-gray-250 dark:focus-visible:bg-gray-700"
      }`}
      disabled={disabled}
      onClick={onClick}
    >
      <Icon className="w-5 h-5" />
    </button>
  );
});

type FilePickerPathViewProps = {
  model: FilePickerModel;
  controller: FilePickerController;
};

/**
 * Displays the segments of the current path in the file picker or an editor to manually enter a path to navigate to.
 */
const FilePickerPathView = observer((props: FilePickerPathViewProps) => {
  const [currentView, setCurrentView] = useState<"segments" | "editor">("segments");
  return currentView === "segments" ? (
    <FilePickerPathSegments
      {...props}
      switchToPathEditor={() => {
        setCurrentView("editor");
      }}
    />
  ) : (
    <FilePickerPathEditor
      {...props}
      switchToPathSegments={() => {
        setCurrentView("segments");
      }}
    />
  );
});

type FilePickerPathSegmentsProps = {
  model: FilePickerModel;
  controller: FilePickerController;
  switchToPathEditor(): void;
};

/**
 * Displays the segments of the current path in the file picker, and allows the for navigating
 * to a different location by clicking on a segment.
 */
const FilePickerPathSegments = observer((props: FilePickerPathSegmentsProps) => {
  // Get a reference to the container with the path segments, so we can scroll to it
  const scrollContainerRef = React.useRef<HTMLDivElement>(null);

  // Scroll the last path segment into view on every render
  useEffect(() => {
    scrollContainerRef.current?.querySelector("[data-path-segment]:last-child")?.scrollIntoView({
      behavior: "smooth",
      block: "center",
      inline: "center",
    });
  });

  const { model, controller } = props;

  return (
    <div className="file-picker-path-segments w-full flex ml-2 min-w-0">
      <div
        ref={scrollContainerRef}
        className="flex-grow no-scrollbars flex gap-1 items-center bg-gray-200 dark:bg-gray-700 text-primary rounded-lg h-10 px-3 overflow-y-hidden"
        onClick={(event) => {
          // Switch to the path editor when the path segments background is clicked directly
          if (event.target === event.currentTarget) {
            props.switchToPathEditor();
          }
        }}
      >
        {model.currentPathSegments.map((segment) => {
          return (
            <div key={segment.path} className="inline-flex items-center flex-shrink-0" data-path-segment>
              <button
                className="hover:bg-gray-300 rounded-lg hover:text-black inline-flex h-10 px-1 items-center"
                onClick={() => {
                  controller.changePathWithHistory(segment.path);
                }}
              >
                {segment.name}
              </button>
              <IconChevron className="w-4 h-4 -mx-px text-icon" />
            </div>
          );
        })}
      </div>
    </div>
  );
});

type FilePickerPathEditorProps = {
  model: FilePickerModel;
  controller: FilePickerController;
  switchToPathSegments(newPath?: string): void;
};

/**
 * Displays a text input and button to manually enter a path to navigate to.
 */
const FilePickerPathEditor = observer((props: FilePickerPathEditorProps) => {
  const { model, controller, switchToPathSegments } = props;
  const [newPath, setNewPath] = useState(model.currentPath);

  const goButtonDisabled = newPath.length === 0;

  return (
    <div className="file-picker-path-editor w-full flex ml-2 min-w-0">
      <input
        autoFocus
        type="text"
        spellCheck={false}
        className="bg-gray-200 dark:bg-gray-700 text-primary rounded-lg h-10 px-3 w-full outline-none border-2 border-transparent focus:border-blue-600"
        value={newPath}
        onChange={(event) => setNewPath(event.target.value)}
        onKeyDown={(event) => {
          if (event.key === "Escape") {
            event.stopPropagation();
            switchToPathSegments();
          } else if (event.key === "Enter") {
            if (newPath.length > 0) {
              controller.changePathWithHistory(newPath);
              switchToPathSegments();
            }
          }
        }}
        onBlur={(event) => {
          // Close the editor (and go back to the segments view) on blur if the newly focused element is not the "Go" button below
          if ((event.relatedTarget as HTMLButtonElement)?.dataset.id !== "file-picker-go-button") {
            switchToPathSegments();
          }
        }}
      />
      <button
        data-id="filepicker-go-button"
        className={` rounded-lg h-full px-2 ml-0.5 ${
          goButtonDisabled
            ? "opacity-40"
            : "hover:bg-gray-250 dark:hover:bg-gray-700 focus-visible:bg-gray-250 focus-visible:text-black"
        }`}
        title="Go to path entered"
        disabled={goButtonDisabled}
        onClick={() => {
          controller.changePathWithHistory(newPath);
          switchToPathSegments();
        }}
      >
        <IconArrowForward className="w-5 h-5" />
      </button>
    </div>
  );
});

type FilePickerSidebarProps = {
  model: FilePickerModel;
  controller: FilePickerController;
};

/**
 * Displays the file picker sidebar, which contains the list of recent paths and quick paths.
 */
const FilePickerSidebar = observer((props: FilePickerSidebarProps) => {
  return (
    <div className="file-picker-sidebar w-full flex flex-col">
      <FilePickerSidebarGroup
        title="Recent paths"
        emptyMessage="No recent paths"
        path={props.model.recentPaths}
        pickerCurrentPath={props.model.currentPath}
        onPathChange={props.controller.changePathWithHistory}
      />
      <FilePickerSidebarGroup
        title="Quick paths"
        emptyMessage="No quick paths"
        path={props.model.quickPaths}
        pickerCurrentPath={props.model.currentPath}
        onPathChange={props.controller.changePathWithHistory}
      />
    </div>
  );
});

type FilePickerSidebarGroupProps = {
  title: string;
  emptyMessage: string;
  path: FilePickerPath[];
  pickerCurrentPath: string;
  onPathChange(newPath: string): void;
};

/**
 * Expands a given path to its absolute path.
 * For `~`, it expands to the home directory
 * For `.`, it expands to the current working directory.
 */
const expandPath = (p: string) => {
  const homeDir = "/home/nubots"; // Replace with the actual user's home directory in your application
  const cwd = "/home/nubots/NUbots/nusight2"; // Replace with the actual current working directory in your application

  if (p.startsWith("~")) {
    return homeDir + p.slice(1);
  } else if (p === ".") {
    return cwd;
  } else if (p.startsWith("./")) {
    return cwd + p.slice(1);
  } else {
    return p;
  }
};

/**
 * Displays a group of paths in the sidebar with a header.
 */
const FilePickerSidebarGroup = observer((props: FilePickerSidebarGroupProps) => {
  return (
    <div className="file-picker-sidebar-group mb-5 last:mb-0 border-t first:border-0 border-gray-300 dark:border-gray-800 pt-1 flex-grow">
      <div className="font-semibold px-3 uppercase py-2">{props.title}</div>
      {props.path.length === 0 ? (
        <div className="px-3 text-gray-500">{props.emptyMessage}</div>
      ) : (
        props.path.map(({ path, name }) => {
          const expandedPath = expandPath(path);
          const isActive = expandPath(props.pickerCurrentPath) === expandedPath;

          return (
            <button
              key={path}
              title={path}
              className={`flex text-left items-center px-3 py-1 min-w-0 w-full ${
                isActive
                  ? "bg-gray-250 dark:bg-gray-800 hover:bg-gray-300 dark:hover:bg-gray-700"
                  : "hover:bg-gray-250 dark:hover:bg-gray-800"
              }`}
              onClick={() => props.onPathChange(path)}
            >
              <IconFolder className="w-4 h-4 mr-2 shrink-0" /> <span className="truncate flex-grow w-0">{name}</span>
            </button>
          );
        })
      )}
    </div>
  );
});
type FilePickerListProps = {
  model: FilePickerModel;
  controller: FilePickerController;
};

const filePickerListColumns = [
  { label: "Name", key: "name" },
  { label: "Date Modified", key: "dateModified" },
  { label: "Size", key: "size" },
] as const;

/** Converts a pixel value to a rem value, with the rem unit appended */
function rem(pxValue: number) {
  return `${pxValue / 16}rem`;
}

/**
 * Displays the column headers and list of files and folders in the current directory,
 * showing any errors for the file picker (e.g. path not found).
 */
const FilePickerList = observer((props: FilePickerListProps) => {
  const { model, controller } = props;

  // Column sizes for the file list: name, size, date modified
  const gridTemplateColumns = `minmax(${rem(212)}, 1fr) minmax(${rem(188)}, ${rem(200)}) minmax(${rem(92)}, ${rem(
    140,
  )})`;

  return (
    <>
      <div
        className="file-picker-list grid gap-1 border-b mb-1 sticky top-0 border-gray-300 dark:border-gray-800"
        style={{ gridTemplateColumns }}
      >
        {filePickerListColumns.map(({ label, key }) => (
          <button
            key={key}
            className={`inline-flex items-center pt-1 pb-1 px-3 font-semibold cursor-pointer select-none hover:bg-gray-250 hover:text-black ${
              key === "size" ? "justify-end" : ""
            }`}
            onClick={() => controller.changeSort(key)}
          >
            {label}
            {model.sortEntries.column === key ? (
              <IconChevron
                className={`ml-1 w-4 h-4 ${model.sortEntries.direction === "asc" ? "-rotate-90" : "rotate-90"}`}
              />
            ) : null}
          </button>
        ))}
      </div>

      {model.error ? (
        <div className="w-full text-center py-8 text-gray-500">
          <IconError className="inline-block w-7 h-7" />
          <div className="mt-2">{model.error}</div>
        </div>
      ) : model.currentPathEntries ? (
        <FilePickerListItems model={model} controller={controller} gridTemplateColumns={gridTemplateColumns} />
      ) : null}
    </>
  );
});

type FilePickerListItemsProps = {
  model: FilePickerModel;
  controller: FilePickerController;
  gridTemplateColumns: string;
};

/**
 * Displays the list of files and folders in the current directory,
 * and allows for selecting files and navigating to other directories.
 */
const FilePickerListItems = observer((props: FilePickerListItemsProps) => {
  const { model, gridTemplateColumns, controller } = props;
  const entries = model.currentPathEntriesSorted;

  // Reset the last selected file index when the entries change
  useEffect(() => {
    controller.setLastSelectedFileIndex(undefined);
  }, [entries]);

  // Return early if there are no entries for the current path
  if (!entries || entries.length === 0) {
    return <div className="w-full text-center py-8 text-gray-500">No matching files in this directory</div>;
  }

  return (
    <>
      {entries.map((entry, index) => {
        const EntryIcon = entry.type === "file" ? IconFile : IconFolder;
        const entrySize = entry.type === "file" && entry.size !== undefined ? formatFileSize(entry.size) : "--";
        const entryIsSelected = model.selectedFiles.includes(entry.path);

        return (
          <button
            key={entry.name}
            className={`grid gap-1 w-full items-center text-left py-2 mt-0.5 outline-0 focus-visible:outline-1 ${
              entryIsSelected ? "hover:bg-gray-250 dark:hover:bg-gray-700" : "hover:bg-gray-250 dark:hover:bg-gray-700"
            }`}
            style={{ gridTemplateColumns }}
            data-modal-autofocus={index === 0 ? true : null}
            onClick={(event) => {
              if (entry.type === "directory") {
                controller.changePathWithHistory(entry.path);
              } else {
                controller.updateSelection({
                  targetEntryIndex: index,
                  selectMultiple: event.ctrlKey || event.metaKey, // Check the meta key to support Command+Click on macOS
                  selectRange: event.shiftKey,
                });
              }
            }}
          >
            <span className="flex items-center px-3">
              <EntryIcon className="w-4 h-4 flex-shrink-0" />
              <span className="pl-2 w-full truncate" title={entry.name}>
                {entry.name}
              </span>
            </span>
            <span className="px-3">{entry.dateModified ? formatFileTimestamp(entry.dateModified) : ""}</span>
            <span className="text-right px-3">{entrySize}</span>
          </button>
        );
      })}
    </>
  );
});

/**
 * Format the given file timestamp for display, in the format `dd/mm/yyyy hh:mm am/pm`
 */
function formatFileTimestamp(timestamp: Date) {
  const dd = String(timestamp.getDate()).padStart(2, "0");
  const mm = String(timestamp.getMonth() + 1).padStart(2, "0"); // January is 0, so we add 1
  const yyyy = timestamp.getFullYear();

  const hh = String(timestamp.getHours() % 12).padStart(2, "0");
  const min = String(timestamp.getMinutes()).padStart(2, "0");
  const amPm = timestamp.getHours() >= 12 ? "PM" : "AM";

  return `${dd}/${mm}/${yyyy} ${hh}:${min} ${amPm}`;
}

/**
 * Format the given file size for display, adding the 'B' byte suffix with the appropriate SI prefix.
 */
function formatFileSize(size: number) {
  const [value, unit] = siUnit(size, "B");
  return `${value.toFixed(2)} ${unit}`;
}
