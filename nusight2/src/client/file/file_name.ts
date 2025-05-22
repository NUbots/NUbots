import { PathSegments } from "./path_segments";

/** Represents a file name, including its extension and base name */
export class FileName {
  /** The full name of the file */
  readonly full: string;

  /**
   * The name of the file excluding the extension.
   *
   * For example, the base of file name `file.nbs` is `file`
   */
  readonly base: string;

  /**
   * The extension of the file name, with the leading dot.
   *
   * For example, the extension of file name `file.nbs` is `.nbs`
   */
  readonly extension: string;

  /**
   * Create a `FileName` instance from the given string or `PathSegments` instance.
   * If given a `PathSegments` instance, its last segment will be used as the file name.
   *
   * @param path Can be one of: file name string, full path string, or `PathSegments` instance
   */
  constructor(path: string | PathSegments) {
    this.full = parseFullName(path);

    const extensionIdx = this.full.lastIndexOf(".");
    const hasExtension = extensionIdx > 0;

    this.base = hasExtension ? this.full.slice(0, extensionIdx) : this.full;
    this.extension = hasExtension ? this.full.slice(extensionIdx) : "";
  }

  toString() {
    return this.full;
  }
}

function parseFullName(path: string | PathSegments) {
  // If given a PathSegments instance, use its last segment as the file name
  if (path instanceof PathSegments) {
    return path.last;
  }
  // If given a path string (indicated by presence of `\` or `/`),
  // parse it into segments and use the last segment as the file name
  else if (path.includes("\\") || path.includes("/")) {
    return new PathSegments(path).last;
  }
  // Otherwise, assume the given string is the file name
  else {
    return path;
  }
}
