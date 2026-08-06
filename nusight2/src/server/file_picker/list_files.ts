import { FilesRequestTypeEnum } from "@proto/message/eye/File";
import fs from "fs";
import os from "os";
import path from "path";

const RequestTypeToEntryNameFilter: Record<FilesRequestTypeEnum, (entryName: string) => boolean> = {
  [FilesRequestTypeEnum.UNKNOWN]: () => false,
  [FilesRequestTypeEnum.DIRECTORY]: () => false,
  [FilesRequestTypeEnum.NBS]: (entryName) => /\.nbs$/i.test(entryName),
};

export interface FileEntry {
  type: "directory" | "file";
  name: string;
  path: string;
  size: number;
  dateModified: Date;
}

/** Get a list of matching files and folders in the given directory */
export async function listFiles(rawDirectory: string, type: FilesRequestTypeEnum) {
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

  const isMatchingEntryName = RequestTypeToEntryNameFilter[type];

  const entries: FileEntry[] = [];
  const entryNames = await fs.promises.readdir(directory);

  for (const entryName of entryNames) {
    try {
      const fullPath = path.join(directory, entryName);
      const stats = await fs.promises.stat(fullPath);
      const isDirectory = stats.isDirectory();

      if (isDirectory || isMatchingEntryName?.(entryName)) {
        entries.push({
          type: isDirectory ? "directory" : "file",
          name: entryName,
          path: fullPath,
          size: stats.size,
          dateModified: stats.mtime,
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
