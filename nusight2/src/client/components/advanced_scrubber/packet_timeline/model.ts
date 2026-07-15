import { NbsScrubberModel } from "@components/nbs_scrubbers/model";
import { TimelineCameraView, TimelineDataPoint } from "@components/timeline/model";
import { CompressedImage } from "@proto/message/output/CompressedImage";
import { MessageInstance } from "@shared/messages/types";
import { NbsTimestamp } from "@shared/time/timestamp";
import { action, observable } from "mobx";

import { MessageTypeId } from "../model";

const messageColors = [
  "#ef4444",
  "#f97316",
  "#f59e0b",
  "#eab308",
  "#84cc16",
  "#22c55e",
  "#10b981",
  "#14b8a6",
  "#06b6d4",
  "#0ea5e9",
  "#3b82f6",
  "#6366f1",
  "#8b5cf6",
  "#a855f7",
  "#d946ef",
  "#ec4899",
  "#f43f5e",
];

/** Get a color for a message type */
export function getMessageColor(messageName: string) {
  // Simple hash of message name to use as index of color so that a message
  // name always has the same color
  const charCodes = [...messageName].map((val) => val.charCodeAt(0));
  const index = charCodes.reduce((a, b) => a + b, 0) % messageColors.length;
  return messageColors[index];
}

/** Get an icon to represent a message type */
export const messageNameToIcon: Record<string, string> = {
  // Icon for unknown message types
  Unknown: "question_mark",
  [CompressedImage.typeName]: "image",
  // Default icon for all other message types
  default: "data_object",
} as const;

export interface MessageViewModel {
  /** Message type info */
  type: MessageTypeId;
  /** Timestamp of the message */
  timestamp: NbsTimestamp;
  /** Size of the message in bytes*/
  size: number;
  /** The message itself */
  message: MessageInstance | Uint8Array;
}

export class PacketTimelineModel {
  @observable view: TimelineCameraView;
  @observable scrubber: NbsScrubberModel;
  @observable.ref selectedMessage: MessageViewModel | undefined;
  @observable.shallow highlightedDataPoints: TimelineDataPoint[] = [];

  constructor(scrubber: NbsScrubberModel, view: TimelineCameraView) {
    this.scrubber = scrubber;
    this.view = view;
  }

  @action
  update(scrubber: NbsScrubberModel, view: TimelineCameraView) {
    this.scrubber = scrubber;
    this.view = view;
  }
}
