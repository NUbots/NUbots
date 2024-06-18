import { FileDialogEntry, FileDialogModel, parsePathSegments } from "../model";

export function createFakeFileDialogModel() {
  return FileDialogModel.of({
    currentPath: JSON.parse(localStorage.getItem("nusight:last_selected_path") ?? `"/"`),
    currentPathEntries: undefined,
    recentPaths: JSON.parse(localStorage.getItem("nusight:recently_selected_paths") ?? "[]"),
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
}

export function sampleFileSystem(): { [key: string]: FileDialogEntry[] } {
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
      sampleEntry("directory", "/nusight2/src/client/components/file_dialog"),
      sampleEntry("directory", "/nusight2/src/client/components/nbs_scrubber"),
      sampleEntry("file", "/nusight2/src/client/components/common.tsx"),
      sampleEntry("file", "/nusight2/src/client/components/common.css"),
    ],
    "/nusight2/src/client/components/file_tree": [],
    "/nusight2/src/client/components/file_dialog": [
      sampleEntry("file", "/nusight2/src/client/components/file_dialog/file_dialog.stories.tsx"),
      sampleEntry("file", "/nusight2/src/client/components/file_dialog/view.tsx"),
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

function sampleEntry(type: "file" | "directory", path: string): FileDialogEntry {
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
