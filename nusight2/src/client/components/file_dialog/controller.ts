import { action } from "mobx";

import { PathSegments } from "../../file/path_segments";

import { FileDialogEntry, parsePathSegments } from "./model";
import { FileDialogModel } from "./model";
import { FileDialogNavigationHistory } from "./model";
import { FileDialogPath } from "./model";
import { FileDialogSort } from "./model";

export class FileDialogController {
  model: FileDialogModel;

  constructor(model: FileDialogModel) {
    this.model = model;
  }

  static of(opts: { model: FileDialogModel }) {
    return new FileDialogController(opts.model);
  }

  /** Toggle visibility of the dialog */
  @action
  toggle = () => {
    this.model.isShown = !this.model.isShown;

    // If we are hiding the dialog, reset state
    if (!this.model.isShown) {
      this.model.error = "";
      this.model.selectedEntries = [];
      this.model.currentPath = JSON.parse(localStorage.getItem("nusight:last_selected_path") ?? '"."');
      this.model.currentPathEntries = undefined;
      this.model.navigationHistory = { back: [], forward: [] };
    }
  };

  /** Set the path currently shown in the dialog */
  @action
  setCurrentPath = (path: string) => {
    this.model.lastVisitedPath = new PathSegments(this.model.currentPath);

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
  setCurrentPathEntries = (entries: FileDialogEntry[]) => {
    this.model.currentPathEntries = entries;
  };

  /** Set the path currently shown in the dialog to the given path and update the navigation history */
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
      // and that's handled in the FileDialogNavButtons component.
      forward: [],
    });

    // Navigate to the new path
    this.setCurrentPath(newPath);
  };

  /** Change the sort of entries in the dialog */
  @action
  changeSort = (column: FileDialogSort["column"]) => {
    // Sort by the new column
    this.model.sortEntries.column = column;

    // If the sort column is the same as the current sort column,
    // reverse the sort order. Otherwise, we're switching the sort
    // from one column to another, so set the order to ascending.
    this.model.sortEntries.direction =
      this.model.sortEntries.column === column ? (this.model.sortEntries.direction === "asc" ? "desc" : "asc") : "asc";
  };

  /** Set the dialog's currently selected files */
  @action
  setSelectedEntries = (entries: FileDialogEntry[]) => {
    this.model.selectedEntries = entries;
  };

  /**
   * Set the dialog's last selected file index. This is the index of the file
   * that was last clicked, in the list of entries for the current path.
   */
  @action
  setLastSelectedEntryIndex = (index?: number) => {
    this.model.lastSelectedEntryIndex = index;
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

    // Clear the last visited path to prevent having both a highlighted and selected path visible at the same time
    this.model.lastVisitedPath = new PathSegments(this.model.currentPath);

    const targetEntry = this.model.currentPathEntriesSorted[targetEntryIndex];

    // Our new list of selected entries
    let newSelectedEntries: FileDialogEntry[] = [];

    // If the current list of selected entries is empty, replace it with a list containing the clicked entry
    if (this.model.selectedEntries.length === 0) {
      newSelectedEntries = [targetEntry];
    }
    // Handle the case where we're selecting a range (the entry was Shift clicked)
    else if (selectRange) {
      // If we don't have a last selected entry index, then there's no range to select
      if (this.model.lastSelectedEntryIndex === undefined) {
        newSelectedEntries = [targetEntry];
      }
      // Otherwise, select the range of entries from the last clicked index to the current one
      else {
        const firstIndex = Math.min(this.model.lastSelectedEntryIndex, targetEntryIndex);
        const lastIndex = Math.max(this.model.lastSelectedEntryIndex, targetEntryIndex);
        newSelectedEntries = this.model.currentPathEntriesSorted.slice(firstIndex, lastIndex + 1);
      }
    }
    // Handle the case where we're selecting multiple (the entry was Ctrl clicked)
    else if (selectMultiple) {
      // If the clicked entry is already selected, then unselect it by removing it from the list
      if (this.model.selectedEntries.includes(targetEntry)) {
        newSelectedEntries = this.model.selectedEntries.filter((entry) => entry !== targetEntry);
      }
      // Otherwise, add the clicked entry to the list of selected entries
      else {
        newSelectedEntries = [...this.model.selectedEntries, targetEntry];
      }
    }
    // Handle the case where the entry was clicked without Ctrl or Shift
    else {
      newSelectedEntries = [targetEntry];
    }

    // Update the list of selected entries
    this.setSelectedEntries(newSelectedEntries);

    // Update the last selected entry index
    this.setLastSelectedEntryIndex(targetEntryIndex);
  };

  /**
   * Update the dialog's list of recent paths. These will be automatically
   * saved to local storage, to persist across page reloads.
   */
  @action
  setRecentPaths = (paths: FileDialogPath[]) => {
    this.model.recentPaths = paths;

    localStorage.setItem("nusight:recently_selected_paths", JSON.stringify(paths));

    if (paths.length > 0) {
      localStorage.setItem("nusight:last_selected_path", JSON.stringify(paths[0].path));
    }
  };

  /** Add the given directory paths to the list of recent paths shown in the sidebar */
  @action
  addRecentPaths = (directories: FileDialogPath[]) => {
    // Convert the existing recent paths into a map of path to { name, path } objects
    const recentPathsMap = new Map<string, FileDialogPath>();
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

  /** Update the list of recent paths from the currently selected files in the dialog */
  @action
  updateRecentPathsFromSelection = () => {
    // Do nothing if we have no selected files
    if (this.model.selectedEntries.length === 0) {
      return;
    }

    // Get the parent directory of each selected file
    const directoryPaths = this.model.selectedEntries.map((file) => {
      const segments = parsePathSegments(file.path);
      return segments[segments.length - 2].path; // the second last path segment is the parent directory
    });

    // Remove duplicates and map each path string to a { name, path } object
    const directories = Array.from(new Set(directoryPaths)).map((directory) => {
      const segments = parsePathSegments(directory);
      return segments[segments.length - 1]; // the last segment has the full path
    });

    this.addRecentPaths(directories);
  };

  /** Set the dialog navigation history, for backward and forward navigation. */
  @action
  setNavigationHistory = (history: FileDialogNavigationHistory) => {
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
   * Update the dialog's error message. Will clear the currently shown error
   * if the given an empty string. Otherwise will show the error message.
   */
  @action
  setError = (error: string) => {
    this.model.error = error;
  };
}
