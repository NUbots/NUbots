import { computed, observable } from "mobx";

import { PathSegments } from "../../file/path_segments";

export type FileDialogEntry = {
  type: "file" | "directory";
  name: string;
  path: string;
  /** Size in bytes for files, 0 for directories */
  size: number;
  dateModified: Date;
};

/** Generic path information: a name and full path. */
export type FileDialogPath = {
  /** Last entry in the path: the name of the file or directory */
  name: string;
  /** The full path */
  path: string;
};

/** A path segment description: the name of the segment and the full path up to that name. */
export type FileDialogPathSegment = {
  name: string;
  path: string;
};

export type FileDialogSort = {
  column: "name" | "dateModified" | "size";
  direction: "asc" | "desc";
};

export type FileDialogNavigationHistory = {
  /** Paths in the back history, from oldest to newest */
  back: string[];
  /** Paths in the forward history, from oldest to newest */
  forward: string[];
};

type EntryComparison = (a: FileDialogEntry, b: FileDialogEntry) => number;

const entryComparison: Record<string, EntryComparison> = {
  name: (a, b) => {
    return a.name.localeCompare(b.name, "en", { numeric: true, sensitivity: "base" });
  },
  dateModified: (a, b) => {
    return a.dateModified.valueOf() - b.dateModified.valueOf();
  },
  size: (a, b) => {
    return a.size - b.size;
  },
} as const;

export class FileDialogModel {
  /** Whether or not the file dialog is shown */
  @observable accessor isShown: boolean;

  /** Error to show in the file dialog, e.g. when attempting to navigate to a non-existent path */
  @observable accessor error: string;

  /** The current path of the file dialog */
  @observable accessor currentPath: string;

  /** The entries (files and folders) at the current path */
  @observable accessor currentPathEntries: FileDialogEntry[] | undefined;

  /** The previously visited path, to highlight where we have come from */
  @observable accessor lastVisitedPath: PathSegments;

  /** How to sort entries for display */
  @observable accessor sortEntries: FileDialogSort;

  /** List of quick paths to show in the sidebar */
  @observable accessor quickPaths: FileDialogPath[];

  /** List of recent paths to show in the sidebar */
  @observable accessor recentPaths: FileDialogPath[];

  /** Currently selected entries of the current path */
  @observable accessor selectedEntries: FileDialogEntry[];

  /** The index of the last selected file in the current path entries */
  @observable accessor lastSelectedEntryIndex: number | undefined;

  /** The navigation history of the file dialog: keeps track of the paths that
   *  were visited in the back and forward directions */
  @observable accessor navigationHistory: FileDialogNavigationHistory;

  constructor(opts: {
    isShown: boolean;
    error: string;
    currentPath: string;
    currentPathEntries?: FileDialogEntry[];
    quickPaths: FileDialogPath[];
    recentPaths: FileDialogPath[];
    selectedEntries: FileDialogEntry[];
  }) {
    this.isShown = opts.isShown;
    this.error = opts.error;
    this.currentPath = opts.currentPath;
    this.currentPathEntries = opts.currentPathEntries;
    this.lastVisitedPath = new PathSegments(this.currentPath);
    this.quickPaths = opts.quickPaths;
    this.recentPaths = opts.recentPaths;
    this.selectedEntries = opts.selectedEntries;

    this.sortEntries = { column: "name", direction: "asc" };
    this.navigationHistory = { back: [], forward: [] };
  }

  static of(
    opts: {
      isShown?: boolean;
      error?: string;
      currentPath?: string;
      currentPathEntries?: FileDialogEntry[];
      quickPaths?: FileDialogPath[];
      recentPaths?: FileDialogPath[];
      selectedEntries?: FileDialogEntry[];
    } = {},
  ) {
    return new FileDialogModel({
      isShown: false,
      error: "",
      currentPath: "",
      currentPathEntries: undefined,
      quickPaths: [
        { name: "/", path: "/" },
        { name: "Home", path: "~" },
        { name: "NUsight CWD", path: "." },
      ],
      recentPaths: [],
      selectedEntries: [],
      ...opts,
    });
  }

  /** Full paths to the currently selected files in the dialog */
  @computed
  get selectedFilePaths(): ReadonlyArray<string> {
    return this.selectedEntries.filter((entry) => entry.type === "file").map((file) => file.path);
  }

  /** Full paths to the currently selected directory entries in the dialog */
  @computed
  get selectedDirectoryPaths(): ReadonlyArray<string> {
    return this.selectedEntries.filter((entry) => entry.type === "directory").map((directory) => directory.path);
  }

  /** List of segments (name, path) in the current path */
  @computed
  get currentPathSegments(): FileDialogPathSegment[] {
    return parsePathSegments(this.currentPath);
  }

  /** List of the current path entries, sorted by the current sort */
  @computed
  get currentPathEntriesSorted(): FileDialogEntry[] | undefined {
    if (!this.currentPathEntries) {
      return undefined;
    }

    if (this.currentPathEntries.length === 0) {
      return [];
    }

    const sortBy = this.sortEntries.column;
    const direction = this.sortEntries.direction;

    const compare = entryComparison[sortBy];

    function sorter(a: FileDialogEntry, b: FileDialogEntry) {
      return direction === "asc" ? compare(a, b) : -compare(a, b);
    }

    // Sort the directories
    const directories = this.currentPathEntries.filter((entry) => entry.type === "directory").sort(sorter);

    // Sort the files
    const files = this.currentPathEntries.filter((entry) => entry.type === "file").sort(sorter);

    // Put directories first, then files
    return [...directories, ...files];
  }

  /** Summary of the selected files, to show at the bottom of the dialog */
  @computed
  get selectedFilesSummary(): string {
    return this.selectedFilePaths.length === 1
      ? this.selectedFilePaths[0]
      : `${this.selectedFilePaths[0]} + ${this.selectedFilePaths.length - 1} ${
          this.selectedFilePaths.length === 2 ? "file" : "files"
        }`;
  }

  /** Summary of the selected directory paths, to show at the bottom of the dialog */
  @computed
  get selectedDirectoriesSummary(): string {
    return this.selectedDirectoryPaths.length === 1
      ? this.selectedDirectoryPaths[0]
      : `${this.selectedDirectoryPaths[0]} + ${this.selectedDirectoryPaths.length - 1} ${
          this.selectedDirectoryPaths.length === 2 ? "path" : "paths"
        }`;
  }
}

/**
 * Parse a file path into its component segments with each segment having a name
 * and a full path. For example, the path `/foo/bar/baz.txt` would be parsed
 * into the following segments:
 *
 * ```
 * [
 *   { name: '/', path: '/' },
 *   { name: 'foo', path: '/foo' },
 *   { name: 'bar', path: '/foo/bar' },
 *   { name: 'baz.txt', path: '/foo/bar/baz.txt' }
 * ]
 * ```
 */
export function parsePathSegments(path: string): FileDialogPathSegment[] {
  // Ignore paths with only whitespace
  if (path.trim().length === 0) {
    return [];
  }

  // Normalise the path
  let pathNormal = path
    .replace(/\\/g, "/") // Windows slashes to Unix slashes
    .replace(/\/\//g, "/"); // Double consecutive slashes to single slashes

  // Remove trailing slash, but only if the path is not the root
  if (pathNormal.trim() !== "/") {
    pathNormal = pathNormal.replace(/\/$/, "");
  }

  return (
    pathNormal
      // Split into segments
      .split(/\//)
      // Remove all empty segments except the first one (which will be the root segment if the path starts with `/`)
      .filter((segmentName, i) => i === 0 || segmentName.length > 0)
      // Map each segment to an object with name and full path
      .map((segmentName, i, allSegmentNames) => {
        return {
          name: segmentName.length === 0 ? "/" : segmentName, // Only empty segment is the root, name it '/'
          path:
            segmentName.length === 0
              ? "/" // Only empty segment is the root
              : i === 0 && /:/.test(segmentName)
              ? segmentName + "/" // If the first segment has a colon then it's a Windows root, so add a trailing slash
              : allSegmentNames.slice(0, i + 1).join("/"), // Otherwise, build the full path from the segments
        };
      })
  );
}
