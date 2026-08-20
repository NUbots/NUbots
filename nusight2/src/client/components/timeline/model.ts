import { action, computed, observable } from "mobx";

export interface TimelineDataPoint {
  timestamp: bigint;
  color?: string;
}

export interface TimelineCameraView {
  centre: bigint;
  range: bigint;
}

export interface TimelineMarkerModel {
  /** The timestamp of the marker */
  timestamp: bigint;
  /** Text to display beside the marker head */
  text?: string;
  /** Color of the marker */
  color: string;
  /** Scale of the marker */
  scale: number;
  /** Whether to hide the tail section of the marker */
  hideTail?: boolean;
  /** Whether to clamp the marker to the view bounds */
  clampToView?: boolean;
  onMouseDown?: (marker: TimelineMarkerModel) => void;
  onMouseUp?: (marker: TimelineMarkerModel) => void;
  onMouseMove?: (time: bigint, marker: TimelineMarkerModel) => void;
  onMouseEnter?: (marker: TimelineMarkerModel) => void;
  onMouseLeave?: (marker: TimelineMarkerModel) => void;
}

export interface HighlightRangeModel {
  start: bigint;
  end: bigint;
  color: string;
  borderColor?: string;
  hideFlags?: boolean;
  hideBody?: boolean;
  onMouseDown?: (side: "start" | "end", range: HighlightRangeModel) => void;
  onMouseUp?: (side: "start" | "end", range: HighlightRangeModel) => void;
  onMouseMove?: (side: "start" | "end", time: bigint, range: HighlightRangeModel) => void;
  onMouseEnter?: (side: "start" | "end", range: HighlightRangeModel) => void;
  onMouseLeave?: (side: "start" | "end", range: HighlightRangeModel) => void;
}

export class TimelineModel {
  view: TimelineCameraView;
  @observable markers: TimelineMarkerModel[] = [];
  @observable highlightRanges: HighlightRangeModel[] = [];

  constructor(view: TimelineCameraView) {
    this.view = view;
  }

  /**
   * Adds a marker to this timeline model.
   * This returns a callback to remove the marker from this model.
   */
  @action
  addMarker(marker: TimelineMarkerModel) {
    this.markers.push(marker);
    return action(() => {
      const idx = this.markers.indexOf(marker);
      if (idx !== -1) {
        this.markers.splice(idx, 1);
      }
    });
  }

  /**
   * Adds a highlight range to this timeline model.
   * This returns a callback to remove the highlight range from this model.
   */
  @action
  addHighlightRange(range: HighlightRangeModel) {
    this.highlightRanges.push(range);
    return action(() => {
      const idx = this.highlightRanges.indexOf(range);
      if (idx !== -1) {
        this.highlightRanges.splice(idx, 1);
      }
    });
  }

  /** The min and max timestamps of the current view */
  @computed
  get viewBounds() {
    const halfViewRange = this.view.range / 2n;
    return { min: this.view.centre - halfViewRange, max: this.view.centre + halfViewRange };
  }
}
