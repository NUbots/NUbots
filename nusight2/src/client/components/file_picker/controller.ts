import { action } from "mobx";

import { FilePickerEntry, parsePathSegments } from "./model";
import { FilePickerModel } from "./model";
import { FilePickerNavigationHistory } from "./model";
import { FilePickerPath } from "./model";
import { FilePickerSort } from "./model";

export class FilePickerController {
  model: FilePickerModel;

  constructor(model: FilePickerModel) {
    this.model = model;
  }

  static of(opts: { model: FilePickerModel }) {
    return new FilePickerController(opts.model);
  }

  /** Toggle visibility of the picker */
  @action
  toggle = () => {
    this.model.isShown = !this.model.isShown;

    // If we are hiding the picker, reset state
    if (!this.model.isShown) {
      this.model.error = "";
      this.model.selectedFiles = [];
      this.model.currentPath = JSON.parse(localStorage.getItem("nusight:last_picked_path") ?? '"."');
      this.model.currentPathEntries = undefined;
      this.model.navigationHistory = { back: [], forward: [] };
    }
  };

  /** Set the path currently shown in the picker */
  @action
  setCurrentPath = (path: string) => {
    // Clear the current path entries if the new path is different.
    // We keep the current entries if the paths are the same to avoid flickering
    // when we clear the entries and then re-add them.
    if (path !== this.model.currentPath) {
      this.model.currentPathEntries = undefined;
    }

    this.model.currentPath = path;
  };

  /** Set the entries (i.e. files and folders) for the current path */
  @action
  setCurrentPathEntries = (entries: FilePickerEntry[]) => {
    this.model.currentPathEntries = entries;
  };

  /** Set the path currently shown in the picker to the given path and update the navigation history */
  @action
  changePathWithHistory = (newPath: string) => {
    // Do nothing if the new path is the same as the current path
    if (newPath === this.model.currentPath) {
      return;
    }

    this.setNavigationHistory({
      back: [...this.model.navigationHistory.back, this.model.currentPath],
      // Reset forward history when we change paths.
      // We keep the forward history only when we navigate using the Back and Forward buttons,
      // and that's handled in the FilePickerNavButtons component.
      forward: [],
    });

    // Navigate to the new path
    this.setCurrentPath(newPath);
  };

  /** Change the sort of entries in the picker */
  @action
  changeSort = (column: FilePickerSort["column"]) => {
    // Sort by the new column
    this.model.sortEntries.column = column;

    // If the sort column is the same as the current sort column,
    // reverse the sort order. Otherwise, we're switching the sort
    // from one column to another, so set the order to ascending.
    this.model.sortEntries.direction =
      this.model.sortEntries.column === column ? (this.model.sortEntries.direction === "asc" ? "desc" : "asc") : "asc";
  };

  /** Set the picker's currently selected files */
  @action
  setSelectedFiles = (paths: string[]) => {
    this.model.selectedFiles = paths;
  };

  /**
   * Set the picker's last selected file index. This is the index of the file
   * that was last clicked, in the list of entries for the current path.
   */
  @action
  setLastSelectedFileIndex = (index?: number) => {
    this.model.lastSelectedFileIndex = index;
  };

  /**
   * Update the list of selected files when an entry in the list of entries is clicked,
   * allowing Ctrl+Click to select multiple files, and Shift+Click to select a range of files.
   */
  @action
  updateSelection = ({
    targetEntryIndex,
    selectMultiple,
    selectRange,
  }: {
    targetEntryIndex: number;
    selectMultiple: boolean;
    selectRange: boolean;
  }) => {
    if (!this.model.currentPathEntriesSorted) {
      return;
    }

    const targetEntry = this.model.currentPathEntriesSorted[targetEntryIndex];

    // Our new list of selected files
    let newSelectedFiles: string[] = [];

    // If the current list of selected files is empty, replace it with a list containing the clicked file
    if (this.model.selectedFiles.length === 0) {
      newSelectedFiles = [targetEntry.path];
    }
    // Handle the case where we're selecting a range (the file was Shift clicked)
    else if (selectRange) {
      // If we don't have a last selected file index, then there's no range to select
      if (this.model.lastSelectedFileIndex === undefined) {
        newSelectedFiles = [targetEntry.path];
      }
      // Otherwise, select the range of files from the last clicked index to the current one
      else {
        const firstIndex = Math.min(this.model.lastSelectedFileIndex, targetEntryIndex);
        const lastIndex = Math.max(this.model.lastSelectedFileIndex, targetEntryIndex);
        newSelectedFiles = this.model.currentPathEntriesSorted
          .slice(firstIndex, lastIndex + 1)
          .map((file) => file.path);
      }
    }
    // Handle the case where we're selecting multiple (the file was Ctrl clicked)
    else if (selectMultiple) {
      // If the clicked file is already selected, then unselect it by removing it from the list
      if (this.model.selectedFiles.includes(targetEntry.path)) {
        newSelectedFiles = this.model.selectedFiles.filter((file) => file !== targetEntry.path);
      }
      // Otherwise, add the clicked file to the list of selected files
      else {
        newSelectedFiles = [...this.model.selectedFiles, targetEntry.path];
      }
    }
    // Handle the case where the file was clicked without Ctrl or Shift
    else {
      newSelectedFiles = [targetEntry.path];
    }

    // Update the list of selected files
    this.setSelectedFiles(newSelectedFiles);

    // Update the last selected file index
    this.setLastSelectedFileIndex(targetEntryIndex);
  };

  /**
   * Update the picker's list of recent paths. These will be automatically
   * saved to local storage, to persist across page reloads.
   */
  @action
  setRecentPaths = (paths: FilePickerPath[]) => {
    this.model.recentPaths = paths;

    localStorage.setItem("nusight:recently_picked_paths", JSON.stringify(paths));

    if (paths.length > 0) {
      localStorage.setItem("nusight:last_picked_path", JSON.stringify(paths[0].path));
    }
  };

  /** Update the list of recent paths from the currently selected files in the picker */
  @action
  updateRecentPathsFromSelection = () => {
    // Do nothing if we have no selected files
    if (this.model.selectedFiles.length === 0) {
      return;
    }

    // Get the parent directory of each selected file
    const directoryPaths = this.model.selectedFiles.map((file) => {
      const segments = parsePathSegments(file);
      return segments[segments.length - 2].path; // the second last path segment is the parent directory
    });

    // Remove duplicates and map each path string to a { name, path } object
    const directories = Array.from(new Set(directoryPaths)).map((directory) => {
      const segments = parsePathSegments(directory);
      return segments[segments.length - 1]; // the last segment has the full path
    });

    // Convert the existing recent paths into a map of path to { name, path } objects
    const recentPathsMap = new Map<string, FilePickerPath>();
    this.model.recentPaths.forEach((pathInfo) => {
      recentPathsMap.set(pathInfo.path, pathInfo);
    });

    // Convert the recent paths map keys (the full path strings) to a set,
    // for easy has() and delete() operations
    const recentPathsSet = new Set(recentPathsMap.keys());

    // Remove from the set of recent paths any paths that match a directory we are selecting from.
    // The new directories we're selecting from will be added back to the set, at the start (see below).
    for (const directory of directories) {
      if (recentPathsSet.has(directory.path)) {
        recentPathsSet.delete(directory.path);
      }
    }

    // Update the recent paths list with the new directories we selected from added to the start
    this.setRecentPaths([...directories, ...Array.from(recentPathsSet).map((path) => recentPathsMap.get(path)!)]);
  };

  /** Set the picker navigation history, for backward and forward navigation. */
  @action
  setNavigationHistory = (history: FilePickerNavigationHistory) => {
    this.model.navigationHistory = history;
  };

  /** Navigate to the previous path in the navigation history */
  @action
  navigateBackward = () => {
    if (this.model.navigationHistory.back.length === 0) {
      return;
    }

    // Get the last path in the back history
    const lastPath = this.model.navigationHistory.back[this.model.navigationHistory.back.length - 1];

    this.setNavigationHistory({
      // Remove the last path from the back history, as we're about to go to it
      back: this.model.navigationHistory.back.slice(0, -1),
      // New forward history is the existing forward history, plus the current path we're about to leave
      forward: [...this.model.navigationHistory.forward, this.model.currentPath],
    });

    // Navigate to the last path in the back history
    this.setCurrentPath(lastPath);
  };

  /** Navigate to the next path in the navigation history */
  @action
  navigateForward = () => {
    if (this.model.navigationHistory.forward.length === 0) {
      return;
    }

    // Get the last path in the forward history
    const lastPath = this.model.navigationHistory.forward[this.model.navigationHistory.forward.length - 1];

    this.setNavigationHistory({
      // New back history is the existing back history, plus the current path we're about to leave
      back: [...this.model.navigationHistory.back, this.model.currentPath],
      // Remove the last path from the forward history, as we're about to go to it
      forward: this.model.navigationHistory.forward.slice(0, -1),
    });

    // Navigate to the last path in the forward history
    this.setCurrentPath(lastPath);
  };

  /** Navigate to the parent path */
  @action
  navigateUpward = () => {
    // Less than two segments means we're at the root and can't go any higher
    if (this.model.currentPathSegments.length < 2) {
      return;
    }

    // New path is the second-last segment's path
    const newPath = this.model.currentPathSegments[this.model.currentPathSegments.length - 2].path;

    if (newPath === this.model.currentPath) {
      return;
    }

    this.setNavigationHistory({
      // New back history is the existing back history, plus the current path we're about to leave
      back: [...this.model.navigationHistory.back, this.model.currentPath],
      // Reset forward history when we change paths without using the Back and Forward buttons
      forward: [],
    });

    // Navigate to the new path
    this.setCurrentPath(newPath);
  };

  /**
   * Update the picker's error message. Will clear the currently shown error
   * if the given an empty string. Otherwise will show the error message.
   */
  @action
  setError = (error: string) => {
    this.model.error = error;
  };
}
