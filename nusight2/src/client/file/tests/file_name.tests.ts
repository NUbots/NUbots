import { describe, expect, it } from "vitest";
import { describe, expect, it } from "vitest";

import { FileName } from "../file_name";
import { PathSegments } from "../path_segments";

describe("FileName", () => {
  describe("with Posix-style paths", () => {
    it("can be created from a path string", () => {
      expect(new FileName("/a/path/to/a/directory/").full).toBe("directory");
      expect(new FileName("/a/path/to/a/file.nbs").full).toBe("file.nbs");
      expect(new FileName("/a/path/to/a/directory/").full).toBe("directory");
      expect(new FileName("/a/path/to/a/file").full).toBe("file");
      expect(new FileName("/file").full).toBe("file");
      expect(new FileName("file").full).toBe("file");
      expect(new FileName("/").full).toBe("");
      expect(new FileName("").full).toBe("");
    });

    it("can be created from a PathSegments instance", () => {
      expect(new FileName(new PathSegments("/a/path/to/a/directory/")).full).toBe("directory");
      expect(new FileName(new PathSegments("/a/path/to/a/file.nbs")).full).toBe("file.nbs");
      expect(new FileName(new PathSegments("/a/path/to/a/directory/")).full).toBe("directory");
      expect(new FileName(new PathSegments("/a/path/to/a/file")).full).toBe("file");
      expect(new FileName(new PathSegments("/file")).full).toBe("file");
      expect(new FileName(new PathSegments("file")).full).toBe("file");
      expect(new FileName(new PathSegments("/")).full).toBe("");
      expect(new FileName(new PathSegments("")).full).toBe("");
    });
  });

  describe("with Windows-style paths", () => {
    it("can be created from a path string", () => {
      expect(new FileName("C:\\a\\path\\to\\a\\directory\\").full).toBe("directory");
      expect(new FileName("C:\\a\\path\\to\\a\\file.nbs").full).toBe("file.nbs");
      expect(new FileName("C:\\a\\path\\to\\a\\directory\\").full).toBe("directory");
      expect(new FileName("C:\\a\\path\\to\\a\\file").full).toBe("file");
      expect(new FileName("C:\\file").full).toBe("file");
      expect(new FileName("C:\\").full).toBe("C:");
    });

    it("can be created from a PathSegments instance", () => {
      expect(new FileName(new PathSegments("C:\\a\\path\\to\\a\\directory\\")).full).toBe("directory");
      expect(new FileName(new PathSegments("C:\\a\\path\\to\\a\\file.nbs")).full).toBe("file.nbs");
      expect(new FileName(new PathSegments("C:\\a\\path\\to\\a\\directory\\")).full).toBe("directory");
      expect(new FileName(new PathSegments("C:\\a\\path\\to\\a\\file")).full).toBe("file");
      expect(new FileName(new PathSegments("C:\\file")).full).toBe("file");
      expect(new FileName(new PathSegments("C:\\")).full).toBe("C:");
    });
  });

  it(".base gets the filename without the extension", () => {
    expect(new FileName("file.nbs.idx").base).toBe("file.nbs");
    expect(new FileName("file.nbs").base).toBe("file");
    expect(new FileName("file").base).toBe("file");
    expect(new FileName(".nbs.idx").base).toBe(".nbs");
    expect(new FileName(".nbs").base).toBe(".nbs");
    expect(new FileName("").base).toBe("");
  });

  // Designed to match Node's `path.extname()` https://nodejs.org/api/path.html#pathextnamepath
  it(".extension gets the extension without the rest of the name", () => {
    expect(new FileName("file.nbs.idx").extension).toBe(".idx");
    expect(new FileName("file.nbs").extension).toBe(".nbs");
    expect(new FileName("file").extension).toBe("");
    expect(new FileName("file.").extension).toBe(".");
    expect(new FileName(".nbs.idx").extension).toBe(".idx");
    expect(new FileName(".nbs").extension).toBe("");
    expect(new FileName("").extension).toBe("");
  });

  it(".full gets the full file name", () => {
    expect(new FileName("file.nbs.idx").full).toBe("file.nbs.idx");
    expect(new FileName("file.nbs").full).toBe("file.nbs");
    expect(new FileName("file").full).toBe("file");
    expect(new FileName(".nbs.idx").full).toBe(".nbs.idx");
    expect(new FileName(".nbs").full).toBe(".nbs");
    expect(new FileName("").full).toBe("");
  });
});
