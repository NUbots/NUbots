import type { NbsTimestamp } from "nbsdecoder.js";

export interface NbsScrubber {
  /** The ID of the scrubber, unique across all scrubbers in a session */
  id: number;

  /** The name of the scrubber, used as the scrubber's peer name */
  name: string;

  /** The earliest timestamp across all files in the scrubber */
  start: NbsTimestamp;

  /** The latest timestamp across all files in the scrubber */
  end: NbsTimestamp;

  /** The current playback state of the scrubber */
  playbackState: "paused" | "playing" | "ended" | "unknown";

  /**
   * The playback speed of the scrubber.
   * Used as a power of 2, so 0 is 1x, 1 is 2x, -1 is 0.5x, etc.
   */
  playbackSpeed: number;

  /** Whether the scrubber restarts when it reaches the end of playback */
  playbackRepeat: boolean;
}
