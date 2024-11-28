import { describe, expect, it } from "vitest";
import { describe, expect, it } from "vitest";

import { PathSegments } from "../path_segments";

describe("PathSegments (with Posix-style paths)", () => {
  it(".separator gets the separator of the path", () => {
    expect(new PathSegments("/path/to/a/directory/").separator).toBe("/");
    expect(new PathSegments("/file").separator).toBe("/");
    expect(new PathSegments("/").separator).toBe("/");

    // Defaults to Posix-style separator for paths without a separator
    expect(new PathSegments("file").separator).toBe("/");
    expect(new PathSegments("").separator).toBe("/");
  });

  it(".segments gets a list of segments in the path", () => {
    expect(new PathSegments("/path/to/a/directory/").segments).toEqual(["", "path", "to", "a", "directory"]);
    expect(new PathSegments("/path/to/a/file.nbs").segments).toEqual(["", "path", "to", "a", "file.nbs"]);
    expect(new PathSegments("path/to/a/directory/").segments).toEqual(["path", "to", "a", "directory"]);
    expect(new PathSegments("path/to/a/file").segments).toEqual(["path", "to", "a", "file"]);
    expect(new PathSegments("/file").segments).toEqual(["", "file"]);
    expect(new PathSegments("file").segments).toEqual(["file"]);
    expect(new PathSegments("/").segments).toEqual([""]);
    expect(new PathSegments("").segments).toEqual([]);
  });

  it(".length gets the number of segments in the path", () => {
    expect(new PathSegments("/path/to/a/directory/").length).toBe(5);
    expect(new PathSegments("/path/to/a/file.nbs").length).toBe(5);
    expect(new PathSegments("path/to/a/directory/").length).toBe(4);
    expect(new PathSegments("path/to/a/file").length).toBe(4);
    expect(new PathSegments("/file").length).toBe(2);
    expect(new PathSegments("file").length).toBe(1);
    expect(new PathSegments("/").length).toBe(1);
    expect(new PathSegments("").length).toBe(0);
  });

  it(".first gets the first non-empty segment in the path", () => {
    expect(new PathSegments("/path/to/a/file.nbs").first).toBe("path");
    expect(new PathSegments("path/to/a/file").first).toBe("path");
    expect(new PathSegments("/file").first).toBe("file");
    expect(new PathSegments("file").first).toBe("file");
    expect(new PathSegments("/").first).toBe("");
    expect(new PathSegments("").first).toBe("");
  });

  it(".last gets the last segment in the path", () => {
    expect(new PathSegments("/path/to/a/directory/").last).toBe("directory");
    expect(new PathSegments("/path/to/a/file.nbs").last).toBe("file.nbs");
    expect(new PathSegments("path/to/a/directory/").last).toBe("directory");
    expect(new PathSegments("path/to/a/file").last).toBe("file");
    expect(new PathSegments("/file").last).toBe("file");
    expect(new PathSegments("file").last).toBe("file");
    expect(new PathSegments("/").last).toBe("");
    expect(new PathSegments("").last).toBe("");
  });

  it(".parent gets all but the last segment in the path", () => {
    expect(new PathSegments("/path/to/a/directory/").parent).toBe("/path/to/a");
    expect(new PathSegments("/path/to/a/file").parent).toBe("/path/to/a");
    expect(new PathSegments("path/to/a/directory/").parent).toBe("path/to/a");
    expect(new PathSegments("path/to/a/file").parent).toBe("path/to/a");
    expect(new PathSegments("/file").parent).toBe("/");
    expect(new PathSegments("file").parent).toBe("");
    expect(new PathSegments("").parent).toBe("");
  });

  it(".slice() creates a new instance from a sub-section of the path", () => {
    const withLeadingSeparator = new PathSegments("/path/to/a/directory/");
    const withoutLeadingSeparator = new PathSegments("path/to/a/directory/");

    // Slicing from the start
    expect(withLeadingSeparator.slice(0, 3).path).toBe("/path/to");
    expect(withoutLeadingSeparator.slice(0, 3).path).toBe("path/to/a");

    // Slicing in the middle
    expect(withLeadingSeparator.slice(1, 4).path).toBe("path/to/a");
    expect(withoutLeadingSeparator.slice(1, 4).path).toBe("to/a/directory");

    // Slicing from the end with negative indices
    expect(withLeadingSeparator.slice(2, -1).path).toBe("to/a");
    expect(withoutLeadingSeparator.slice(2, -1).path).toBe("a");
  });

  it(".exclude() creates a new instance that excludes a sub-section of the path", () => {
    const path = new PathSegments("/long/path/to/a/file");

    // Exclude from the start
    expect(path.exclude("/long/path").path).toBe("/to/a/file");
    expect(path.exclude("long/path").path).toBe("/to/a/file");

    // Exclude from the middle
    expect(path.exclude("/to/a").path).toBe("/long/path/file");
    expect(path.exclude("to/a").path).toBe("/long/path/file");

    // Exclude from the end
    expect(path.exclude("/file").path).toBe("/long/path/to/a");
    expect(path.exclude("file").path).toBe("/long/path/to/a");
  });

  it(".join() creates a new instance that joins the path with another path", () => {
    const path = new PathSegments("/long/path/");

    // Can join with a string
    expect(path.join("to/a/file").path).toBe("/long/path/to/a/file");
    expect(path.join("/to/a/file").path).toBe("/long/path/to/a/file");

    // Can join with another PathSegments instance
    expect(path.join(new PathSegments("to/a/file")).path).toBe("/long/path/to/a/file");
    expect(path.join(new PathSegments("/to/a/file")).path).toBe("/long/path/to/a/file");
  });

  it(".isParentOf() checks if the path is a parent of the given path", () => {
    const path = new PathSegments("/path/to");

    // A parent if the given path (string or PathSegments) starts with the path
    expect(path.isParentOf("/path/to/a/file")).toBe(true);
    expect(path.isParentOf(new PathSegments("/path/to/a/file"))).toBe(true);

    // Not a parent if the given path (string or PathSegments) does not start with the path
    expect(path.isParentOf("/unrelated/file/path")).toBe(false);
    expect(new PathSegments("/unrelated/file/path").isParentOf(path)).toBe(false);

    // A path is not a parent of itself
    expect(path.isParentOf(path)).toBe(false);
  });
});

describe("PathSegments (Windows-style paths)", () => {
  it(".separator gets the separator of the path", () => {
    expect(new PathSegments("C:\\path\\to\\a\\file.nbs").separator).toBe("\\");
    expect(new PathSegments("\\file").separator).toBe("\\");
    expect(new PathSegments("\\").separator).toBe("\\");
  });

  it(".segments gets a list of segments in the path", () => {
    expect(new PathSegments("C:\\path\\to\\a\\directory\\").segments).toEqual(["C:", "path", "to", "a", "directory"]);
    expect(new PathSegments("C:\\path\\to\\a\\file.nbs").segments).toEqual(["C:", "path", "to", "a", "file.nbs"]);
    expect(new PathSegments("\\path\\to\\a\\directory\\").segments).toEqual(["", "path", "to", "a", "directory"]);
    expect(new PathSegments("\\path\\to\\a\\file").segments).toEqual(["", "path", "to", "a", "file"]);
    expect(new PathSegments("path\\to\\a\\directory\\").segments).toEqual(["path", "to", "a", "directory"]);
    expect(new PathSegments("path\\to\\a\\file").segments).toEqual(["path", "to", "a", "file"]);
    expect(new PathSegments("\\file").segments).toEqual(["", "file"]);
    expect(new PathSegments("\\").segments).toEqual([""]);
  });

  it(".length gets the number of segments in the path", () => {
    expect(new PathSegments("C:\\path\\to\\a\\directory\\").length).toBe(5);
    expect(new PathSegments("C:\\path\\to\\a\\file.nbs").length).toBe(5);
    expect(new PathSegments("\\path\\to\\a\\directory\\").length).toBe(5);
    expect(new PathSegments("\\path\\to\\a\\file").length).toBe(5);
    expect(new PathSegments("path\\to\\a\\directory\\").length).toBe(4);
    expect(new PathSegments("path\\to\\a\\file").length).toBe(4);
    expect(new PathSegments("C:\\file").length).toBe(2);
    expect(new PathSegments("\\").length).toBe(1);
  });

  it(".first gets the first non-empty segment in the path", () => {
    expect(new PathSegments("C:\\path\\to\\a\\file").first).toBe("C:");
    expect(new PathSegments("\\path\\to\\a\\file.nbs").first).toBe("path");
    expect(new PathSegments("path\\to\\a\\file.nbs").first).toBe("path");
    expect(new PathSegments("C:\\file").first).toBe("C:");
    expect(new PathSegments("\\file").first).toBe("file");
    expect(new PathSegments("\\").first).toBe("");
    expect(new PathSegments("").first).toBe("");
  });

  it(".last gets the last segment in the path", () => {
    expect(new PathSegments("C:\\path\\to\\a\\directory\\").last).toBe("directory");
    expect(new PathSegments("C:\\path\\to\\a\\file.nbs").last).toBe("file.nbs");
    expect(new PathSegments("\\path\\to\\a\\directory\\").last).toBe("directory");
    expect(new PathSegments("\\path\\to\\a\\file").last).toBe("file");
    expect(new PathSegments("C:\\file").last).toBe("file");
    expect(new PathSegments("C:\\").last).toBe("C:");
    expect(new PathSegments("\\").last).toBe("");
  });

  it(".parent gets all but the last segment in the path", () => {
    expect(new PathSegments("C:\\path\\to\\a\\directory\\").parent).toBe("C:\\path\\to\\a");
    expect(new PathSegments("C:\\path\\to\\a\\file").parent).toBe("C:\\path\\to\\a");
    expect(new PathSegments("\\path\\to\\a\\directory\\").parent).toBe("\\path\\to\\a");
    expect(new PathSegments("\\path\\to\\a\\file").parent).toBe("\\path\\to\\a");
    expect(new PathSegments("path\\to\\a\\directory\\").parent).toBe("path\\to\\a");
    expect(new PathSegments("path\\to\\a\\file").parent).toBe("path\\to\\a");
    expect(new PathSegments("C:\\file").parent).toBe("C:");
  });

  it(".slice() creates a new instance from a sub-section of the path", () => {
    const withRootSegment = new PathSegments("C:\\path\\to\\a\\directory\\");
    const withLeadingSeparator = new PathSegments("\\path\\to\\a\\directory\\");
    const withoutLeadingSeparator = new PathSegments("path\\to\\a\\directory\\");

    // Slicing from the start
    expect(withRootSegment.slice(0, 3).path).toBe("C:\\path\\to");
    expect(withLeadingSeparator.slice(0, 3).path).toBe("\\path\\to");
    expect(withoutLeadingSeparator.slice(0, 3).path).toBe("path\\to\\a");

    // Slicing in the middle
    expect(withRootSegment.slice(1, 4).path).toBe("path\\to\\a");
    expect(withLeadingSeparator.slice(1, 4).path).toBe("path\\to\\a");
    expect(withoutLeadingSeparator.slice(1, 4).path).toBe("to\\a\\directory");

    // Slicing from the end with negative indices
    expect(withRootSegment.slice(2, -1).path).toBe("to\\a");
    expect(withLeadingSeparator.slice(2, -1).path).toBe("to\\a");
    expect(withoutLeadingSeparator.slice(2, -1).path).toBe("a");
  });

  it(".exclude() creates a new instance that excludes a sub-section of the path", () => {
    const path = new PathSegments("C:\\long\\path\\to\\a\\file");

    // Exclude from the start
    expect(path.exclude("C:\\long\\path").path).toBe("\\to\\a\\file");
    expect(path.exclude("long\\path").path).toBe("C:\\to\\a\\file");

    // Exclude from the middle
    expect(path.exclude("\\to\\a").path).toBe("C:\\long\\path\\file");
    expect(path.exclude("to\\a").path).toBe("C:\\long\\path\\file");

    // Exclude from the end
    expect(path.exclude("\\file").path).toBe("C:\\long\\path\\to\\a");
    expect(path.exclude("file").path).toBe("C:\\long\\path\\to\\a");
  });

  it(".join() creates a new instance that joins the path with another path", () => {
    const path = new PathSegments("C:\\long\\path\\");

    // Can join with a string
    expect(path.join("to\\a\\file").path).toBe("C:\\long\\path\\to\\a\\file");
    expect(path.join("\\to\\a\\file").path).toBe("C:\\long\\path\\to\\a\\file");

    // Can join with another PathSegments instance
    expect(path.join(new PathSegments("to\\a\\file")).path).toBe("C:\\long\\path\\to\\a\\file");
    expect(path.join(new PathSegments("\\to\\a\\file")).path).toBe("C:\\long\\path\\to\\a\\file");
  });

  it(".isParentOf() checks if the path is a parent of the given path", () => {
    const path = new PathSegments("C:\\path\\to");

    // A parent if the given path (string or PathSegments) starts with the path
    expect(path.isParentOf("C:\\path\\to\\a\\file")).toBe(true);
    expect(path.isParentOf(new PathSegments("C:\\path\\to\\a\\file"))).toBe(true);

    // Not a parent if the given path (string or PathSegments) does not start with the path
    expect(path.isParentOf("C:\\unrelated\\file\\path")).toBe(false);
    expect(new PathSegments("C:\\unrelated\\file\\path").isParentOf(path)).toBe(false);

    // A path is not a parent of itself
    expect(path.isParentOf(path)).toBe(false);
  });
});
