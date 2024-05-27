/**
 * Represents a path as a list of segments, for easier manipulation at the level of segments instead of characters.
 *
 * Note that since this class is designed for use on the client where there is no direct filesystem access,
 * it understands only the syntax of a path, not semantics like absolute vs relative, file vs directory, etc.
 */
export class PathSegments {
  /** The type of separator used by this path */
  readonly separator: "\\" | "/";

  /** The full path as a string */
  readonly path: string;

  /**
   * The individual segments of the path. If this is a path that begins with `/` or `\`,
   * the first segment will be empty.
   */
  readonly segments: ReadonlyArray<string>;

  /** The number of segments of the path */
  readonly length: number;

  /**
   * The first non-empty segment of the path. For example:
   *   - first of path `/path/to/file.nbs` is `path`
   *   - first of path `C:\path\to\file.nbs` is `C:`
   */
  readonly first: string;

  /**
   * The last segment of the path.
   *
   * For example, last of path `/path/to/file.nbs` is `file.nbs`
   */
  readonly last: string;

  /**
   * The parent part of this path, defined as all segments except the last.
   *
   * For example, parent of path `/path/to/file.nbs` is `/path/to`
   */
  readonly parent: string;

  /** Create a new `PathSegments` instance from the given path string */
  constructor(path: string) {
    this.separator = path.includes("\\") ? "\\" : "/";

    // Remove double separators and separator at the end. A separator at the start is okay.
    this.segments = path.length > 0 ? path.split(this.separator).filter((segment, i) => i === 0 || segment !== "") : [];

    this.path = this.segments.join(this.separator);
    this.first = this.segments.find((segment) => segment.length > 0) ?? "";
    this.length = this.segments.length;

    this.parent = this.segments.slice(0, -1).join(this.separator);

    // If the parent computed above is empty but the path has segments, the parent is the separator
    // (e.g. for path "/file", the parent is "/" not "")
    if (this.parent === "" && this.segments.length > 1) {
      this.parent = this.separator;
    }

    this.last = this.segments[this.segments.length - 1] ?? "";
  }

  /**
   * Create a new `PathSegments` instance by slicing out a sub-section of this path using segment indices.
   *
   * For example:
   *
   * ```js
   * new PathSegments("/path/to/a/file").slice(0, 3).path // "/path/to"
   * new PathSegments("/path/to/a/file").slice(2).path    // "to/a/file"
   * new PathSegments("/path/to/a/file").slice(-2).path   // "a/file"
   * ```
   */
  slice(start: number, end?: number): PathSegments {
    return new PathSegments(this.segments.slice(start, end).join(this.separator));
  }

  /**
   * Create a new `PathSegments` instance from this path with a sub-section removed.
   *
   * For example:
   *
   * ```js
   * new PathSegments("/path/to/a/file").exclude("/path/to") // "/a/file"
   * ```
   */
  exclude(other: string | PathSegments): PathSegments {
    const a = this;
    let b = typeof other === "string" ? new PathSegments(other) : other;

    // Normalise b to a's separator
    if (a.separator !== b.separator) {
      b = new PathSegments(b.segments.join(a.separator));
    }

    return new PathSegments(a.path.replace(b.path, ""));
  }

  /**
   * Create a new instance of PathSegments by appending the segments of another path to the end of this path.
   *
   * For example:
   *
   * ```js
   * new PathSegments("/path/to").join("/a/file") // "/path/to/a/file"
   * ```
   */
  join(other: string | PathSegments): PathSegments {
    const a = this;
    const b = typeof other === "string" ? new PathSegments(other) : other;

    // Filter empty segments to prevent a result with double separator (//)
    const bSegments = b.segments.filter((segment) => segment.length !== 0);

    return new PathSegments([...a.segments, ...bSegments].join(a.separator));
  }

  /**
   * Check if the current path segments together form a parent of the given path.
   *
   * This is true if the current path is shorter than the given path and all segments
   * of the current path match the corresponding segments of the given path.
   *
   * For example:
   *
   * ```js
   * const parent = new PathSegments("/path/to");
   * const child = new PathSegments("/path/to/a/file");
   *
   * parent.isParentOf(child) // true
   * child.isParentOf(parent) // false
   * ```
   */
  isParentOf(other: string | PathSegments): boolean {
    const a = this;
    const b = typeof other === "string" ? new PathSegments(other) : other;

    // Can't be a parent of a longer/equal length path
    if (a.length >= b.length) {
      return false;
    }

    // Compare segments until a non-matching one is found
    for (let i = 0; i < a.length; i++) {
      if (a.segments[i] !== b.segments[i]) {
        return false;
      }
    }

    return true;
  }

  toString() {
    return this.path;
  }
}
