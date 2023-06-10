import { computed, observable } from "mobx";

export type FilePickerEntry = {
  type: "file" | "directory";
  name: string;
  path: string;
  /** Size in bytes for files, 0 for directories */
  size: number;
  dateModified: Date;
};

/** Generic path information: a name and full path. */
export type FilePickerPath = {
  /** Last entry in the path: the name of the file or directory */
  name: string;
  /** The full path */
  path: string;
};

/** A path segment description: the name of the segment and the full path up to that name. */
export type FilePickerPathSegment = {
  name: string;
  path: string;
};

export type FilePickerSort = {
  column: "name" | "dateModified" | "size";
  direction: "asc" | "desc";
};

export type FilePickerNavigationHistory = {
  /** Paths in the back history, from oldest to newest */
  back: string[];
  /** Paths in the forward history, from oldest to newest */
  forward: string[];
};

export class FilePickerModel {
  /** Whether or not the file picker is shown */
  @observable isShown: boolean;

  /** Error to show in the file picker, e.g. when attempting to navigate to a non-existent path */
  @observable error: string;

  /** The current path of the file picker */
  @observable currentPath: string;

  /** The entries (files and folders) at the current path */
  @observable currentPathEntries?: FilePickerEntry[];

  /** How to sort entries for display */
  @observable sortEntries: FilePickerSort;

  /** List of quick paths to show in the sidebar */
  @observable quickPaths: FilePickerPath[];

  /** List of recent paths to show in the sidebar */
  @observable recentPaths: FilePickerPath[];

  /** Full paths to files currently selected in the file picker */
  @observable selectedFiles: string[];

  /** The index of the last selected file in the current path entries */
  @observable lastSelectedFileIndex?: number;

  /** The navigation history of the file picker: keeps track of the paths that
   *  were visited in the back and forward directions */
  @observable navigationHistory: FilePickerNavigationHistory;

  constructor(opts: {
    isShown: boolean;
    error: string;
    currentPath: string;
    currentPathEntries?: FilePickerEntry[];
    quickPaths: FilePickerPath[];
    recentPaths: FilePickerPath[];
    selectedFiles: string[];
  }) {
    this.isShown = opts.isShown;
    this.error = opts.error;
    this.currentPath = opts.currentPath;
    this.currentPathEntries = opts.currentPathEntries;
    this.quickPaths = opts.quickPaths;
    this.recentPaths = opts.recentPaths;
    this.selectedFiles = opts.selectedFiles;

    this.sortEntries = { column: "name", direction: "asc" };
    this.navigationHistory = { back: [], forward: [] };
  }

  static of(
    opts: {
      isShown?: boolean;
      error?: string;
      currentPath?: string;
      currentPathEntries?: FilePickerEntry[];
      quickPaths?: FilePickerPath[];
      recentPaths?: FilePickerPath[];
      selectedFiles?: string[];
    } = {},
  ) {
    return new FilePickerModel({
      isShown: false,
      error: "",
      currentPath: "",
      currentPathEntries: undefined,
      quickPaths: [],
      recentPaths: [],
      selectedFiles: [],
      ...opts,
    });
  }

  /** List of segments (name, path) in the current path */
  @computed
  get currentPathSegments(): FilePickerPathSegment[] {
    return parsePathSegments(this.currentPath);
  }

  /** List of the current path entries, sorted by the current sort */
  @computed
  get currentPathEntriesSorted(): FilePickerEntry[] | undefined {
    if (!this.currentPathEntries) {
      return undefined;
    }

    if (this.currentPathEntries.length === 0) {
      return [];
    }

    const sortBy = this.sortEntries.column;
    const direction = this.sortEntries.direction;

    function sorter(a: FilePickerEntry, b: FilePickerEntry) {
      const aValue = a[sortBy];
      const bValue = b[sortBy];

      if (aValue < bValue) {
        return direction === "asc" ? -1 : 1;
      }

      if (aValue > bValue) {
        return direction === "asc" ? 1 : -1;
      }

      return 0;
    }

    // Sort the directories
    const directories = this.currentPathEntries.filter((entry) => entry.type === "directory").sort(sorter);

    // Sort the files
    const files = this.currentPathEntries.filter((entry) => entry.type === "file").sort(sorter);

    // Directories first if sorting ASC, files first if sorting DESC
    return direction === "asc" ? [...directories, ...files] : [...files, ...directories];
  }

  /** Summary of the selected files, to show at the bottom of the picker */
  @computed
  get selectedFilesSummary(): string {
    return this.selectedFiles.length === 1
      ? this.selectedFiles[0]
      : `${this.selectedFiles[0]} + ${this.selectedFiles.length - 1} ${
          this.selectedFiles.length === 2 ? "file" : "files"
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
export function parsePathSegments(path: string): FilePickerPathSegment[] {
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
