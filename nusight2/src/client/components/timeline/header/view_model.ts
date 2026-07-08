import { Render2DEventHandlers } from "@client/render2d/event/event_handlers";
import { Render2DMouseEvent } from "@client/render2d/event/mouse_event";
import { Geometry } from "@client/render2d/object/geometry";
import { Group } from "@client/render2d/object/group";
import { Shape } from "@client/render2d/object/shape";
import { flag, line, markerShape, text } from "@components/timeline/geometry";
import { clamp } from "@shared/math/clamp";
import { Transform } from "@shared/math/transform";
import { Vector2 } from "@shared/math/vector2";
import { Timestamp } from "@shared/time/timestamp";
import { action, computed } from "mobx";

import { TimelineController } from "../controller";
import { HighlightRangeModel, TimelineModel } from "../model";
import { TimelineViewModel } from "../view_model";

import { HeaderInput } from "./timeline_header";

const colors = {
  timePoints: "var(--color-auto-contrast-5)",
} as const;

/** Header section of a TimelineView */
export class TimelineHeaderViewModel extends TimelineViewModel {
  private onPointerDown?: HeaderInput;
  private onPointerMove?: HeaderInput;
  private onPointerUp?: HeaderInput;

  constructor(
    model: TimelineModel,
    controller: TimelineController,
    onPointerDown?: HeaderInput,
    onPointerMove?: HeaderInput,
    onPointerUp?: HeaderInput,
  ) {
    super(model, controller);
    this.onPointerDown = onPointerDown;
    this.onPointerMove = onPointerMove;
    this.onPointerUp = onPointerUp;
  }

  @action
  update(args: ConstructorParameters<typeof TimelineHeaderViewModel>) {
    const [model, controller, onPointerDown, onPointerMove, onPointerUp] = args;
    this.model = model;
    this.controller = controller;
    this.onPointerDown = onPointerDown;
    this.onPointerMove = onPointerMove;
    this.onPointerUp = onPointerUp;
  }

  readonly eventHandlers: Render2DEventHandlers = {
    ...this.defaultEventHandlers,
    onMouseMove: action((e) => {
      this.defaultEventHandlers.onMouseMove?.(e);
      this.onPointerMove?.(this.hoveredTime);
    }),
    onMouseUp: action((e) => {
      this.defaultEventHandlers.onMouseUp?.(e);
      this.onPointerUp?.(this.hoveredTime);
    }),
    onMouseDown: action((e) => {
      if (this.onPointerDown) {
        this.onPointerDown(this.hoveredTime);
      } else {
        this.defaultEventHandlers.onMouseDown?.(e);
      }
    }),
  };

  /** Camera transform to convert coordinates to pixel values */
  @computed
  get camera(): Transform {
    return Transform.of({
      scale: Vector2.of(this.canvas.width, -this.canvas.height),
      // Half pixel offset centers coordinates on pixels instead of the
      // border between pixels. This means lines with width 1 will display properly.
      translate: Vector2.of(this.canvas.width / 2 + 0.5, this.canvas.height / 2 - 0.5),
    });
  }

  /** The elements of the timeline header section */
  @computed
  get scene(): Group {
    return Group.of({
      children: [this.timeTicks, this.highlightAreaFlags, this.visibleMarkers, this.outOfBoundsMarkers],
    });
  }

  /**
   * Vertical lines to show time points at set interval.
   * The major lines also show text above them for their time.
   */
  @computed
  get timeTicks(): Group {
    const geometry: Shape<Geometry>[] = [];
    const { width, height } = this.canvas;
    const { major, minor } = this.timeGridPositions;
    const color = colors.timePoints;

    /**
     * Linear smoothing of a value when it is between the given range.
     * Returns 0 when past the min side and 1 when past the max side.
     */
    function smoothBetween(value: number, min: number, max: number) {
      return clamp((value - min) / (max - min), 0, 1);
    }

    /**
     * Get an alpha value from a given x position, allowing the text to fade out
     * as it reaches the edge of the view
     */
    function getFadeOutAlpha(position: number) {
      const transparentPos = 0;
      const opaquePos = 50;
      return (
        smoothBetween(position, transparentPos, opaquePos) *
        smoothBetween(position, width + transparentPos, width - opaquePos)
      );
    }

    const commonOpts = { align: "end", baseline: "hanging", size: "13px" } as const;

    // The minor lines - 10% the height of the header
    minor.forEach(({ position }) =>
      geometry.push(line({ x: position, y: height * 0.9, y2: height, color, alpha: getFadeOutAlpha(position) })),
    );

    // The major lines - 65% the height of the header with a text label for each
    major.forEach(({ time, position }) => {
      const alpha = getFadeOutAlpha(position);
      if (alpha > 0) {
        const textX = position - 5; // Shift the text a little bit to the side of the major line
        const textY = height * 0.3; // Shift the text a little down from the major line
        geometry.push(
          line({ x: position, y: height * 0.35, y2: height, color, alpha }),
          text({ text: Timestamp.toFormattedDate(time), x: textX, y: textY, color, alpha, ...commonOpts }),
        );
      }
    });

    return Group.of({ children: geometry });
  }

  @computed
  private get highlightAreaFlags(): Group {
    // Filter out ranges with the flags hidden
    const visibleRanges = this.model.highlightRanges.filter((range) => !range.hideFlags);
    if (visibleRanges.length === 0) {
      return Group.of();
    }

    // Create the geometry for one side of a highlight range
    const createGeometry = (side: "start" | "end", range: HighlightRangeModel) => {
      return flag({
        x: this.timeToPixel(range[side]),
        y: this.canvas.height,
        direction: side === "start" ? "left" : "right",
        color: range.color,
        outline: range.borderColor,
        eventHandlers: {
          onMouseDown: (e: Render2DMouseEvent) => {
            range.onMouseDown?.(side, range);
            e.stopPropagation();
          },
          onMouseUp: () => range.onMouseUp?.(side, range),
          onMouseMove: () => range.onMouseMove?.(side, this.hoveredTime, range),
          onMouseEnter: () => range.onMouseEnter?.(side, range),
          onMouseLeave: () => range.onMouseLeave?.(side, range),
        },
      });
    };

    // Create a geometry for each side of each visible range
    return Group.of({
      children: visibleRanges.flatMap((range) => [createGeometry("start", range), createGeometry("end", range)]),
    });
  }

  @computed
  private get markerViewModels() {
    const graceRegion = 4; // Number of pixels for marker to be out of view by before being considered out of bounds

    return this.model.markers.map((marker) => {
      const x = this.timeToPixel(marker.timestamp);
      const isInView = x >= -graceRegion && x <= this.canvas.width + graceRegion;
      return { x, isInView, marker };
    });
  }

  @computed
  get visibleMarkers(): Group {
    const y = this.canvas.height;
    const visibleMarkers = this.markerViewModels.filter((m) => m.isInView);

    return Group.of({
      children: visibleMarkers.map(({ x, marker }) => {
        // Create event handlers for the marker
        const eventHandlers: Render2DEventHandlers = {
          onMouseDown: (e: Render2DMouseEvent) => {
            marker.onMouseDown?.(marker);
            e.stopPropagation();
          },
          onMouseUp: () => marker.onMouseUp?.(marker),
          onMouseMove: () => marker.onMouseMove?.(this.hoveredTime, marker),
          onMouseEnter: () => marker.onMouseEnter?.(marker),
          onMouseLeave: () => marker.onMouseLeave?.(marker),
        };

        // If the marker doesn't have any event handlers, ignore pointer events
        const ignorePointerEvents =
          (marker.onMouseDown ||
            marker.onMouseMove ||
            marker.onMouseUp ||
            marker.onMouseEnter ||
            marker.onMouseLeave) === undefined;

        const children: Shape<Geometry>[] = [markerShape({ color: marker.color, eventHandlers, ignorePointerEvents })];
        if (marker.text) {
          children.push(
            text({
              text: marker.text,
              x: -15,
              y: -10,
              color: marker.color,
              align: "end",
              baseline: "middle",
              size: "13px",
            }),
          );
        }

        return Group.of({
          children,
          transform: Transform.of({ translate: { x, y }, scale: { x: marker.scale, y: marker.scale } }),
        });
      }),
    });
  }

  @computed
  get outOfBoundsMarkers(): Group {
    const y = this.canvas.height - 12;
    const outOfBoundsMarkers = this.markerViewModels.filter((m) => m.marker.clampToView && !m.isInView);
    const centrePixel = this.canvas.width / 2;

    // Sort by furthest to the centre of the view so that closest appears on top (rendered last)
    const sortedMarkers = outOfBoundsMarkers.sort((a, b) => Math.abs(b.x - centrePixel) - Math.abs(a.x - centrePixel));

    // Function for scaling markers based on their distance from the view
    function getScale(distance: number) {
      // Scale with negative exponential function to make markers scale fast initially and then more gradually
      // as they get further away.
      // 'power' a lower value increases the range that the markers scale over
      // 'minScale' is the minimum scale that the markers will be scaled to
      // 'base' is the base of the exponential function, changing the steepness of the curve
      const power = 1 / 8000;
      const minScale = 0.7;
      const base = Math.E;
      const exp = Math.pow(base, -(distance * power));

      // Clamp the scale within valid range
      return clamp(exp, minScale, 1);
    }

    return Group.of({
      children: sortedMarkers.map(({ x, marker }) => {
        // Scale down markers as they move further from the view
        const distanceToEdge = x < 0 ? -x : x - this.canvas.width;
        const scale = marker.scale * getScale(distanceToEdge);

        // Rotate and shift the marker up so the base is at the edge of the view
        const transform = Transform.of({
          translate: { x: clamp(x, 0, this.canvas.width), y },
          rotate: x < 0 ? Math.PI / 2 : -Math.PI / 2,
          scale: { x: scale, y: scale },
        });

        // Center the view on the marker if it is clicked
        const eventHandlers: Render2DEventHandlers = {
          onMouseDown: (e: Render2DMouseEvent) => {
            this.controller.setViewCentre(marker.timestamp);
            e.stopPropagation();
          },
        };

        return Group.of({ children: [markerShape({ color: marker.color, eventHandlers })], transform });
      }),
    });
  }
}
