import fs from "fs";
import os from "os";
import path from "path";

export interface NbsScrubberFileEntry {
  type: "directory" | "file";
  name: string;
  path: string;
  size: number;
  dateModified: Date;
}

/** Get a list of NBS files and folders in the given directory */
export async function listNbsFiles(rawDirectory: string) {
  if (rawDirectory.trim().length === 0) {
    throw new Error("Cannot list files for blank path");
  }

  let directory = rawDirectory;

  // Expand relative paths
  if (directory === "~") {
    directory = os.homedir();
  } else if (directory === ".") {
    directory = process.cwd();
  } else if (directory.startsWith("~/")) {
    directory = path.join(os.homedir(), directory.slice(2));
  }

  const entries: NbsScrubberFileEntry[] = [];
  const entryNames = await fs.promises.readdir(directory);

  for (const entryName of entryNames) {
    try {
      const fullPath = path.join(directory, entryName);
      const stats = await fs.promises.stat(fullPath);
      const isDirectory = stats.isDirectory();

      if (isDirectory || /\.nbs$/i.test(entryName)) {
        entries.push({
          type: isDirectory ? "directory" : "file",
          name: entryName,
          path: fullPath,
          size: stats.size,
          // Even though `stats.mtime` is a Date, we reconstruct it date here using the current global Date
          // constructor so we can do instanceof checks in Jest. This is necessary since Jest overrides
          // the global Date constructor, but not the one used by fs.stat() internally, leading to
          // failing instanceof checks when running in Jest.
          dateModified: new Date(stats.mtime),
        });
      }
    } catch (e) {
      // Skip entries that we are unable to stat
      continue;
    }
  }

  return {
    directory,
    entries: entries.sort((a, b) => {
      // Sort entries alphabetically, directories first
      return a.type === b.type ? a.name.localeCompare(b.name) : a.type.localeCompare(b.type);
    }),
  };
}
