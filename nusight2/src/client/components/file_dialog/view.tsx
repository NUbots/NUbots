import React, { useContext, useState } from "react";
import { useEffect } from "react";
import classNames from "classnames";
import { observer } from "mobx-react";

import { siUnit } from "../../base/si_unit";
import { PathSegments } from "../../file/path_segments";
import { Button } from "../button/button";
import { Icon } from "../icon/view";
import { Modal } from "../modal/view";

import { FileDialogController } from "./controller";
import { FileDialogModel } from "./model";
import { FileDialogPath } from "./model";

export interface FileDialogProps {
  title: string;
  model: FileDialogModel;
  controller: FileDialogController;
  children: React.ReactNode;
  type: "file" | "directory";
  onClose(selectedFiles: string[]): void;
}

/**
 * Context set by implementations of the FileDialog.
 *
 * Allows the top level to alter the behaviour of the lower level components, such as changing whether multi-file
 * selection is allowed.
 */
export const FileDialogContext = React.createContext<{
  allowMultiSelect: boolean;
  type: FileDialogProps["type"];
}>({
  allowMultiSelect: true,
  type: "file",
});

/** Displays a modal dialog for picking files */
export const FileDialog = observer((props: FileDialogProps) => {
  const { title, model, controller, children, onClose } = props;

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
        <div className="border-y border-auto py-1 px-2 flex">
          <FileDialogNavButtons model={model} controller={controller} />
          <FileDialogPathView model={model} controller={controller} />
        </div>

        {/*
          Main content: sidebar and file list.
          165px is the height of the header + nav bar + footer. We subtract that from the viewport height to get
          the max height of the content area, to ensure we always have the dialog header, navbar, and footer visible.
        */}
        <div className="flex flex-grow h-[32rem] max-h-[calc(100vh_-_165px)]">
          <div className="border-r border-auto flex w-52 shrink-0 h-full overflow-y-auto">
            <FileDialogSidebar model={model} controller={controller} />
          </div>
          <div className="flex-grow h-full overflow-auto">
            <FileDialogList model={model} controller={controller} />
          </div>
        </div>

        {/* Footer: controls for specific implementation + Cancel button */}
        <div className="border-t border-auto flex items-center justify-end p-4 gap-2">
          {children}
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

type FileDialogNavButtonsProps = {
  model: FileDialogModel;
  controller: FileDialogController;
};

/**  Displays Back, Forward, and Up navigation buttons for the file dialog, and handles navigation */
const FileDialogNavButtons = observer((props: FileDialogNavButtonsProps) => {
  const { model, controller } = props;

  return (
    <>
      <FileDialogNavButton
        icon="arrow_back"
        disabled={model.navigationHistory.back.length === 0}
        onClick={controller.navigateBackward}
      />
      <FileDialogNavButton
        icon="arrow_forward"
        disabled={model.navigationHistory.forward.length === 0}
        onClick={controller.navigateForward}
      />
      <FileDialogNavButton
        icon="arrow_upward"
        disabled={model.currentPathSegments.length < 2}
        onClick={controller.navigateUpward}
      />
    </>
  );
});

type FileDialogNavButtonProps = {
  icon: string;
  disabled: boolean;
  onClick: () => void;
};

/** Displays a single navigation button */
const FileDialogNavButton = observer((props: FileDialogNavButtonProps) => {
  const { icon, disabled, onClick } = props;
  return (
    <button
      className={`px-2 py-1 rounded inline-flex items-center text-auto-icon-button ${
        disabled ? "opacity-40" : "hover:bg-auto-contrast-1 focus-visible:bg-auto-contrast-1"
      }`}
      disabled={disabled}
      onClick={onClick}
    >
      <Icon weight="400">{icon}</Icon>
    </button>
  );
});

type FileDialogPathViewProps = {
  model: FileDialogModel;
  controller: FileDialogController;
};

/**
 * Displays the segments of the current path in the file dialog or an editor to manually enter a
 * path to navigate to.
 */
const FileDialogPathView = observer((props: FileDialogPathViewProps) => {
  const [currentView, setCurrentView] = useState<"segments" | "editor">("segments");
  return currentView === "segments" ? (
    <FileDialogPathSegments
      {...props}
      switchToPathEditor={() => {
        setCurrentView("editor");
      }}
    />
  ) : (
    <FileDialogPathEditor
      {...props}
      switchToPathSegments={() => {
        setCurrentView("segments");
      }}
    />
  );
});

type FileDialogPathSegmentsProps = {
  model: FileDialogModel;
  controller: FileDialogController;
  switchToPathEditor(): void;
};

/**
 * Displays the segments of the current path in the file dialog, and allows the for navigating
 * to a different location by clicking on a segment.
 */
const FileDialogPathSegments = observer((props: FileDialogPathSegmentsProps) => {
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
        className="flex-grow no-scrollbars flex gap-1 items-center bg-auto-contrast-1 text-auto-primary hover:bg-auto-contrast-2 cursor-text rounded h-10 px-3 overflow-y-hidden"
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
                className="hover:bg-auto-contrast-1 inline-flex h-10 px-1 items-center"
                onClick={() => {
                  controller.changePathWithHistory(segment.path);
                }}
              >
                {segment.name}
              </button>
              <Icon className="text-xl/none text-auto-icon -ml-px -mr-1">chevron_right</Icon>
            </div>
          );
        })}
      </div>
    </div>
  );
});

type FileDialogPathEditorProps = {
  model: FileDialogModel;
  controller: FileDialogController;
  switchToPathSegments(newPath?: string): void;
};

/** Displays a text input and button to manually enter a path to navigate to */
const FileDialogPathEditor = observer((props: FileDialogPathEditorProps) => {
  const { model, controller, switchToPathSegments } = props;
  const [newPath, setNewPath] = useState(model.currentPath);

  const goButtonDisabled = newPath.length === 0;

  return (
    <div className="file-picker-path-editor w-full flex ml-2 min-w-0">
      <input
        autoFocus
        type="text"
        spellCheck={false}
        className="bg-auto-surface-1 text-auto-primary rounded-l h-10 px-3 w-full outline-none border-2 border-transparent focus:border-nusight-500"
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
        className={`bg-auto-contrast-1 text-auto-secondary rounded-r h-full px-2 ml-0.5 inline-flex items-center ${
          goButtonDisabled
            ? "opacity-40"
            : "hover:bg-auto-contrast-2 hover:text-auto-primary focus-visible:bg-nusight-100 focus-visible:text-black"
        }`}
        title="Go to path entered"
        disabled={goButtonDisabled}
        onClick={() => {
          controller.changePathWithHistory(newPath);
          switchToPathSegments();
        }}
      >
        <Icon>arrow_forward</Icon>
      </button>
    </div>
  );
});

type FileDialogSidebarProps = {
  model: FileDialogModel;
  controller: FileDialogController;
};

/** Displays the file dialog sidebar, which contains the list of recent paths and quick paths */
const FileDialogSidebar = observer((props: FileDialogSidebarProps) => {
  return (
    <div className="file-dialog-sidebar w-full flex flex-col">
      <FileDialogSidebarGroup
        title="Recent paths"
        emptyMessage="No recent paths"
        path={props.model.recentPaths}
        dialogCurrentPath={props.model.currentPath}
        onPathChange={props.controller.changePathWithHistory}
      />
      <FileDialogSidebarGroup
        title="Quick paths"
        emptyMessage="No quick paths"
        path={props.model.quickPaths}
        dialogCurrentPath={props.model.currentPath}
        onPathChange={props.controller.changePathWithHistory}
      />
    </div>
  );
});

type FileDialogSidebarGroupProps = {
  title: string;
  emptyMessage: string;
  path: FileDialogPath[];
  dialogCurrentPath: string;
  onPathChange(newPath: string): void;
};

/** Displays a group of paths in the sidebar with a header */
const FileDialogSidebarGroup = observer((props: FileDialogSidebarGroupProps) => {
  return (
    <div className="file-picker-sidebar-group mb-5 last:mb-0 border-t first:border-0 border-auto pt-1 flex-grow">
      <div className="font-semibold px-3 uppercase py-2">{props.title}</div>
      {props.path.length === 0 ? (
        <div className="px-3 text-auto-secondary">{props.emptyMessage}</div>
      ) : (
        props.path.map(({ path, name }) => {
          return (
            <button
              key={path}
              title={path}
              className={`flex text-auto-primary text-left items-center px-3 py-1 min-w-0 w-full ${
                path === props.dialogCurrentPath
                  ? "bg-nusight-200 dark:bg-nusight-900"
                  : "hover:bg-nusight-100 dark:hover:bg-nusight-900/25"
              }`}
              onClick={() => props.onPathChange(path)}
            >
              <Icon className="mr-2 shrink-0" size={20} weight={400} fill>
                folder
              </Icon>
              <span className="truncate flex-grow w-0">{name}</span>
            </button>
          );
        })
      )}
    </div>
  );
});

type FileDialogListProps = {
  model: FileDialogModel;
  controller: FileDialogController;
};

const fileDialogListColumns = [
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
 * showing any errors for the file dialog (e.g. path not found).
 */
const FileDialogList = observer((props: FileDialogListProps) => {
  const { model, controller } = props;

  // Column sizes for the file list: name, size, date modified
  const gridTemplateColumns = `minmax(${rem(212)}, 1fr) minmax(${rem(188)}, ${rem(200)}) minmax(${rem(92)}, ${rem(
    140,
  )})`;

  return (
    <>
      <div
        className="file-picker-list grid gap-1 border-b border-auto mb-1 sticky top-0 bg-auto-surface-2"
        style={{ gridTemplateColumns }}
      >
        {fileDialogListColumns.map(({ label, key }) => (
          <button
            key={key}
            className={`inline-flex items-center pt-1 pb-1 px-3 font-semibold cursor-pointer select-none hover:bg-auto-contrast-1 text-auto-secondary hover:text-auto-primary ${
              key === "size" ? "justify-end" : ""
            }`}
            onClick={() => controller.changeSort(key)}
          >
            {label}
            {model.sortEntries.column === key ? (
              <Icon className="text-xl/none ml-1">
                {model.sortEntries.direction === "asc" ? "expand_less" : "expand_more"}
              </Icon>
            ) : null}
          </button>
        ))}
      </div>

      {model.error ? (
        <div className="w-full text-center py-8 text-auto-secondary">
          <Icon className="inline-block" size={40} weight={400}>
            error
          </Icon>
          <div className="mt-2">{model.error}</div>
        </div>
      ) : model.currentPathEntries ? (
        <FileDialogListItems model={model} controller={controller} gridTemplateColumns={gridTemplateColumns} />
      ) : null}
    </>
  );
});

type FileDialogListItemsProps = {
  model: FileDialogModel;
  controller: FileDialogController;
  gridTemplateColumns: string;
};

/**
 * Displays the list of files and folders in the current directory,
 * and allows for selecting files and navigating to other directories.
 */
const FileDialogListItems = observer((props: FileDialogListItemsProps) => {
  const { model, gridTemplateColumns, controller } = props;
  const entries = model.currentPathEntriesSorted;
  const lastPath = model.lastVisitedPath;

  const { allowMultiSelect, type } = useContext(FileDialogContext);

  // Reset the last selected file index when the entries change
  useEffect(() => {
    controller.setLastSelectedEntryIndex(undefined);
  }, [entries]);

  // Return early if there are no entries for the current path
  if (!entries || entries.length === 0) {
    return <div className="w-full text-center py-8 text-auto-secondary">No matching files in this directory</div>;
  }

  return (
    <>
      {(type === "directory" ? entries.filter((entry) => entry.type === "directory") : entries).map((entry, index) => {
        const entryIcon = entry.type === "file" ? "draft" : "folder";
        const entrySize = entry.type === "file" && entry.size !== undefined ? formatFileSize(entry.size) : "--";
        const entryIsSelected = model.selectedEntries.includes(entry);

        // Highlight the directory leading to the last visited path
        const entrySegments = new PathSegments(entry.path);
        const isParentOfPreviousPath = entrySegments.isParentOf(lastPath) || entry.path === lastPath.path;

        return (
          <button
            key={entry.name}
            className={classNames("grid gap-1 w-full items-center text-left py-2 mt-0.5", {
              "bg-nusight-200 dark:bg-nusight-900": entryIsSelected,
              "hover:bg-nusight-100 dark:hover:bg-nusight-900/25": !entryIsSelected,
              "bg-auto-contrast-1": isParentOfPreviousPath,
            })}
            style={{ gridTemplateColumns }}
            data-focus-container-autofocus={index === 0 ? true : null}
            onClick={(event) => {
              // If we clicked a file entry or the dialog is a directory picker, update our selection
              if (entry.type === "file" || type === "directory") {
                controller.updateSelection({
                  targetEntryIndex: index,
                  selectMultiple: allowMultiSelect && (event.ctrlKey || event.metaKey), // Check the meta key to support Command+Click on macOS,
                  selectRange: allowMultiSelect && event.shiftKey,
                });
              } else {
                controller.changePathWithHistory(entry.path);
              }
            }}
            onDoubleClick={() => {
              if (entry.type === "directory") {
                controller.changePathWithHistory(entry.path);
              }
            }}
          >
            <span className="flex items-center px-3">
              <Icon className="flex-shrink-0" size={20} weight={400} fill={entry.type === "directory"}>
                {entryIcon}
              </Icon>
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

/** Format the given file timestamp for display, in the format `dd/mm/yyyy hh:mm am/pm` */
function formatFileTimestamp(timestamp: Date) {
  const dd = String(timestamp.getDate()).padStart(2, "0");
  const mm = String(timestamp.getMonth() + 1).padStart(2, "0"); // January is 0, so we add 1
  const yyyy = timestamp.getFullYear();

  // The ternary ensures it outputs '12' instead of '00' for 12pm while also displaying '12' for 12am
  const hh = String(timestamp.getHours() % 12 == 0 ? 12 : timestamp.getHours() % 12).padStart(2, "0");
  const min = String(timestamp.getMinutes()).padStart(2, "0");
  const amPm = timestamp.getHours() >= 12 ? "PM" : "AM";

  return `${dd}/${mm}/${yyyy} ${hh}:${min} ${amPm}`;
}

/** Format the given file size for display, adding the 'B' byte suffix with the appropriate SI prefix */
function formatFileSize(size: number) {
  const [value, unit] = siUnit(size, "B");
  return `${value.toFixed(2)} ${unit}`;
}
