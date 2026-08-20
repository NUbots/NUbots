import { ClusteringTree, TreeData, TreeDataCluster } from "@client/data_structures/cluster_tree";
import { Render2DEventHandlers } from "@client/render2d/event/event_handlers";
import { Render2DMouseEvent } from "@client/render2d/event/mouse_event";
import { Geometry } from "@client/render2d/object/geometry";
import { Group } from "@client/render2d/object/group";
import { Shape } from "@client/render2d/object/shape";
import { clamp } from "@shared/math/clamp";
import { Transform } from "@shared/math/transform";
import { Vector2 } from "@shared/math/vector2";
import { action, computed, observable } from "mobx";
import { createTransformer } from "mobx-utils";

import { TimelineController } from "../controller";
import { line, rectangle, text } from "../geometry";
import { HighlightRangeModel, TimelineDataPoint } from "../model";
import { TimelineModel } from "../model";
import { TimelineViewModel } from "../view_model";

const colors = {
  content: "currentColor",
  outlineInner: "var(--color-auto-ring-inner)",
  outlineOuter: "var(--color-auto-ring-outer)",
  dataPointBorder: "rgba(0,0,0,0.2)",
  dataClusterText: "rgba(0,0,0,0.4)",
} as const;

const DefaultAppearance = {
  dataPoint: { width: 15, height: 18 },
  connector: { height: 4 },
} as const;

export class TimelineRowViewModel extends TimelineViewModel {
  private readonly innerOutlineWidth = 1.5;
  private readonly outerOutlineWidth = 3.5;

  @observable.ref private data: TimelineDataPoint[];
  @observable.shallow private connectors: TimelineDataPoint[];
  @observable.shallow private highlightedDataPoints: TimelineDataPoint[];

  @observable dataPointWidth: number;
  @observable dataPointHeight: number;
  @observable connectorHeight: number;

  private onDataPointClick?: (dataPoint: TimelineDataPoint, e: Render2DMouseEvent) => void;

  constructor(
    model: TimelineModel,
    controller: TimelineController,
    data?: TimelineDataPoint[],
    connectorPoints?: TimelineDataPoint[],
    highlightedPoints?: TimelineDataPoint[],
    dataPointWidth?: number,
    dataPointHeight?: number,
    connectorHeight?: number,
    onDataPointClick?: (dataPoint: TimelineDataPoint, e: Render2DMouseEvent) => void,
  ) {
    super(model, controller);
    this.data = data ?? [];
    this.connectors = connectorPoints ?? [];
    this.highlightedDataPoints = highlightedPoints ?? [];
    this.dataPointWidth = dataPointWidth ?? DefaultAppearance.dataPoint.width;
    this.dataPointHeight = dataPointHeight ?? DefaultAppearance.dataPoint.height;
    this.connectorHeight = connectorHeight ?? DefaultAppearance.connector.height;
    this.onDataPointClick = onDataPointClick;
  }

  @action
  update(args: ConstructorParameters<typeof TimelineRowViewModel>) {
    const [
      model,
      controller,
      data,
      connectorPoints,
      highlightedPoints,
      dataPointWidth,
      dataPointHeight,
      connectorHeight,
      onDataPointClick,
    ] = args;
    this.model = model;
    this.controller = controller;
    this.data = data ?? [];
    this.connectors = connectorPoints ?? [];
    this.highlightedDataPoints = highlightedPoints ?? [];
    this.dataPointWidth = dataPointWidth ?? DefaultAppearance.dataPoint.width;
    this.dataPointHeight = dataPointHeight ?? DefaultAppearance.dataPoint.height;
    this.connectorHeight = connectorHeight ?? DefaultAppearance.connector.height;
    this.onDataPointClick = onDataPointClick;
  }

  /** Camera transform to convert coordinates to pixel values */
  @computed
  get camera(): Transform {
    return Transform.of({
      scale: Vector2.of(this.canvas.width, -this.canvas.height),
      // Half pixel offset centers coordinates on pixels instead of the
      // border between pixels. This means lines with width 1 will display properly.
      translate: Vector2.of(this.canvas.width / 2 + 0.5, 0),
    });
  }

  @computed
  get scene(): Group {
    return Group.of({
      children: [
        this.verticalGridLines,
        this.connectorsGeometry,
        this.nodesGeometry,
        this.highlightedDataPointsGeometry,
        this.markerTails,
        this.highlightRanges,
      ],
    });
  }

  /** Minimum time gap between data points before they are clustered together */
  @computed
  get minDataPointGap() {
    const nanosPerPixel = this.model.view.range / BigInt(this.canvas.width);
    return BigInt(this.dataPointWidth + 1) * nanosPerPixel;
  }

  /** This row's data as a cluster tree */
  @computed
  get dataClusterTree() {
    return new ClusteringTree((dataPoint) => dataPoint.timestamp, this.data);
  }

  /** Highlighted data points to only include the ones in this row's data */
  @computed
  get highlightedDataPointsInRow() {
    return this.highlightedDataPoints.filter((dataPoint) => this.data.includes(dataPoint));
  }

  /**
   * The visible nodes of each row depending on the zoom level and offset of the view.
   * This merges data points together into a cluster if they are overlapping.
   */
  @computed
  get visibleNodes() {
    const { min, max } = this.model.viewBounds;
    const nodes: TreeData<TimelineDataPoint>[] = [];

    // Get the first node before and after our range
    const [prev, next] = this.dataClusterTree.getBoundingData(min, max, this.minDataPointGap);

    // Add all the nodes in the range
    prev && nodes.push(prev);
    nodes.push(...this.dataClusterTree.getData(this.minDataPointGap, min, max));
    next && nodes.push(next);

    return nodes;
  }

  /** Checks if a data point's corresponding node is currently visible */
  private readonly isNodeVisible = createTransformer((dataPoint: TimelineDataPoint) => {
    return this.visibleNodes.find((node) => node.type === "single" && node.value === dataPoint);
  });

  private getClosestDataPoint(timestamp: bigint): TimelineDataPoint | undefined {
    const [left, right] = this.dataClusterTree.getBoundingData(timestamp, timestamp);

    // Check if there is 0 or 1 closest data points, returning undefined or that
    // data point respectively.
    if (!left || !right) {
      return left ? left.value : right?.value;
    }

    // If there is a data point on both sides return the one with the closest timestamp
    const leftDist = Math.abs(Number(this.hoveredTime - left.value.timestamp));
    const rightDist = Math.abs(Number(this.hoveredTime - right.value.timestamp));
    return leftDist < rightDist ? left.value : right.value;
  }

  /** The connector points filtered to only include changes in color */
  @computed
  get filteredConnectorPoints() {
    const defaultColor = colors.content;
    return this.connectors.filter((dataPoint, i, connectors) => {
      // Always include first and last data points
      if (i === 0 || i === connectors.length - 1) {
        return true;
      }

      const thisColor = dataPoint.color ?? defaultColor;
      const prevColor = connectors[i - 1].color ?? defaultColor;
      return thisColor !== prevColor;
    });
  }

  /** Vertical lines at the time points of the markers from the model */
  @computed
  private get markerTails(): Group {
    const { height } = this.canvas;
    const markersWithTail = this.model.markers.filter((marker) => !marker.hideTail);
    return Group.of({
      children: markersWithTail.map((marker) =>
        line({
          x: this.timeToPixel(marker.timestamp),
          y: -height / 2,
          y2: height / 2,
          color: marker.color,
          width: 2,
          ignorePointerEvents: true,
        }),
      ),
    });
  }

  @computed
  private get highlightRanges(): Group {
    // Filter out ranges with the body hidden
    const visibleRanges = this.model.highlightRanges.filter((range) => !range.hideBody);
    if (visibleRanges.length === 0) {
      return Group.of();
    }

    const { height } = this.canvas;
    const hitBoxWidth = 8;

    // Create the geometry for one side of a highlight range
    const createGeometry = (side: "start" | "end", range: HighlightRangeModel) => {
      return [
        line({
          // Shift the line a little to place it on the outside of the actual range.
          // This means that when the range is 0, both lines will be visible.
          x: this.timeToPixel(range[side]) + (side === "start" ? -0.5 : 0.5),
          y: -height / 2,
          y2: height / 2,
          color: range.borderColor ?? range.color,
          width: 1.5,
          ignorePointerEvents: true,
        }),
        // Rectangle to serve as the hitbox for the range bounds
        rectangle({
          // Set the side of the hitbox on its line so that when the range
          // is 0, the hitboxes don't overlap and both sides are clickable.
          x: this.timeToPixel(range[side]) + (side === "start" ? -hitBoxWidth : 0),
          width: hitBoxWidth,
          y: -height / 2,
          height: height,
          color: "transparent",
          border: "transparent",
          cursor: "ew-resize",
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
        }),
      ];
    };

    return Group.of({
      children: visibleRanges.flatMap((highlightRange) => [
        rectangle({
          x: this.timeToPixel(highlightRange.start),
          width: this.timeToPixel(highlightRange.end) - this.timeToPixel(highlightRange.start),
          y: -height / 2,
          height: height,
          color: highlightRange.color,
          alpha: 0.2,
          border: "transparent",
        }),
        ...createGeometry("start", highlightRange),
        ...createGeometry("end", highlightRange),
      ]),
    });
  }

  /** Vertical lines to show time points at set intervals */
  @computed
  private get verticalGridLines(): Group {
    const children: Shape<Geometry>[] = [];
    const { major, minor } = this.timeGridPositions;
    const { height } = this.canvas;
    const color = "var(--color-auto-contrast-6)";

    minor.forEach((time) =>
      children.push(line({ x: time.position, y: -height / 2, y2: height / 2, color, alpha: 0.2 })),
    );
    major.forEach((time) =>
      children.push(line({ x: time.position, y: -height / 2, y2: height / 2, color, alpha: 0.5 })),
    );

    return Group.of({ children });
  }

  /** Geometry for all the connection lines between data points */
  @computed
  private get connectorsGeometry() {
    const children = this.filteredConnectorPoints.map(this.createConnectorGeometry);
    return Group.of({ children });
  }

  /** Geometry for the nodes (data points and clusters) of this row */
  @computed
  private get nodesGeometry() {
    const children = this.visibleNodes.map(this.createNodeGeometry);
    return Group.of({ children });
  }

  /** Geometry for the nodes that are currently highlighted */
  @computed
  private get highlightedDataPointsGeometry() {
    const children = this.highlightedDataPointsInRow.map(this.createHighlightedDataPointGeometry);
    return Group.of({ children });
  }

  /** Create geometry for the connector from a connector point to the next connector point */
  private readonly createConnectorGeometry = createTransformer((dataPoint: TimelineDataPoint) => {
    const defaultColor = colors.content;
    const height = this.connectorHeight;
    const { width } = this.canvas;

    const nextDataPoint = this.filteredConnectorPoints[this.filteredConnectorPoints.indexOf(dataPoint) + 1];
    if (!nextDataPoint) {
      return Group.of();
    }

    const x1 = clamp(this.timeToPixel(dataPoint.timestamp), 0, width);
    const x2 = clamp(this.timeToPixel(nextDataPoint.timestamp), 0, width);

    const commonOpts = {
      x: x1,
      y: -height / 2,
      width: x2 - x1,
      height,
      color: dataPoint.color ?? defaultColor,
      cursor: "pointer",
    } as const;

    return Group.of({
      children: [
        // Using multiple layers, draw the connector with a dark border
        rectangle(commonOpts),
        rectangle({ ...commonOpts, color: colors.dataPointBorder }),
        rectangle({ ...commonOpts, y: -height / 2 + 1, height: height - 2 }),
      ],
    });
  });

  /** Create geometry for a node containing either a cluster or single data point */
  private readonly createNodeGeometry = (node: TreeData<TimelineDataPoint>) => {
    return node.type === "cluster" ? this.createClusterGeometry(node) : this.createDataPointGeometry(node.value);
  };

  /** Create geometry for a single cluster of data points */
  private readonly createClusterGeometry = createTransformer((node: TreeDataCluster<TimelineDataPoint>): Group => {
    const defaultColor = colors.content;
    const { min, max } = this.model.viewBounds;
    const height = this.dataPointHeight;

    // If outside the view range return empty group
    if (node.max.timestamp < min || node.min.timestamp > max) {
      return Group.of();
    }

    // Timestamp of the left and right of the geometry
    const timestampLeft = node.min.timestamp < min ? min : node.min.timestamp > max ? max : node.min.timestamp;
    const timestampRight = node.max.timestamp < min ? min : node.max.timestamp > max ? max : node.max.timestamp;

    // Positions of the left and right of the geometry
    const x1 = this.timeToPixel(timestampLeft);
    const x2 = this.timeToPixel(timestampRight);

    const children: Shape<Geometry>[] = [];

    const eventHandlers: Render2DEventHandlers | undefined = this.onDataPointClick
      ? {
          onMouseDown: (e) => {
            const closestDataPoint = this.getClosestDataPoint(this.hoveredTime);
            if (closestDataPoint) {
              this.onDataPointClick?.(closestDataPoint, e);
            }
          },
        }
      : undefined;

    const x = x1 - this.dataPointWidth / 2;
    const y = -height / 2;
    const width = x2 - x1 + this.dataPointWidth;
    const color = node.min.color ?? defaultColor;
    const commonOpts = { x, y, width, height, color, cursor: "pointer", borderRadius: "full", eventHandlers } as const;

    // Using multiple layers, draw the cluster with a dark border
    children.push(
      rectangle(commonOpts),
      rectangle({ ...commonOpts, color: colors.dataPointBorder }),
      rectangle({
        ...commonOpts,
        x: x1 - this.dataPointWidth / 2 + 1,
        y: -height / 2 + 1,
        width: x2 - x1 + this.dataPointWidth - 2,
        height: height - 2,
      }),
    );

    if (node.childCount > 5) {
      // Centre position of the centre of the geometry
      const visibleCentre = (x1 + x2) / 2;

      // Timestamp centre and range of visible section of cluster
      const localCentre = (timestampLeft + timestampRight) / 2n;

      // Timestamp centre and range of entire cluster
      const globalCentre = (node.max.timestamp + node.min.timestamp) / 2n;
      const globalRange = node.max.timestamp - node.min.timestamp;

      // Position text to shift it toward global centre without going off screen
      // (x0.8 to stop the text partially going out of view)
      const textPos = visibleCentre - (Number(localCentre - globalCentre) / Number(globalRange)) * (x2 - x1) * 0.8;

      // Calculate frequency of nodes in the cluster
      const frequency = (node.childCount / Number(node.max.timestamp - node.min.timestamp)) * 1e9;
      children.push(
        text({
          text: `${frequency.toFixed(2)}Hz`,
          x: textPos,
          y: 1,
          size: "13px",
          color: colors.dataClusterText,
          baseline: "middle",
        }),
      );
    }

    return Group.of({ children });
  });

  /** Create geometry for a single data point with an outline */
  private readonly createHighlightedDataPointGeometry = createTransformer((dataPoint: TimelineDataPoint) => {
    const width = this.dataPointWidth;
    const height = this.dataPointHeight;

    const x = this.timeToPixel(dataPoint.timestamp);
    const commonOpts = { x, y: -height / 2 + width / 2, y2: height / 2 - width / 2, cap: "round" } as const;

    // If the dataPoint's node is visible create a solid outline, otherwise if the dataPoint is part of a
    // cluster create a dashed outline
    const outline = this.isNodeVisible(dataPoint)
      ? [
          line({ ...commonOpts, width: width + 2 * this.outerOutlineWidth, color: colors.outlineOuter }),
          line({ ...commonOpts, width: width + 2 * this.innerOutlineWidth, color: colors.outlineInner }),
        ]
      : [
          rectangle({
            borderRadius: "full",
            color: colors.outlineInner,
            x: x - width / 2 - this.outerOutlineWidth,
            y: -height / 2 - this.outerOutlineWidth,
            width: width + 2 * this.outerOutlineWidth,
            height: height + 2 * this.outerOutlineWidth,
          }),
          rectangle({
            borderRadius: "full",
            color: "transparent",
            alpha: 0,
            border: colors.outlineOuter,
            borderWidth: 2,
            // Note: borderDashArray is not supported in NUbots BasicAppearance; dashed border renders solid
            borderDashArray: "4",
            x: x - width / 2 - this.outerOutlineWidth + 0.5,
            y: -height / 2 - this.outerOutlineWidth + 0.5,
            width: width + 2 * this.outerOutlineWidth - 1,
            height: height + 2 * this.outerOutlineWidth - 1,
          }),
        ];

    return Group.of({ children: [...outline, this.createDataPointGeometry(dataPoint)] });
  });

  /** Create geometry for a single data point */
  private readonly createDataPointGeometry = createTransformer((dataPoint: TimelineDataPoint) => {
    const defaultColor = colors.content;
    const width = this.dataPointWidth;
    const height = this.dataPointHeight;

    const eventHandlers: Render2DEventHandlers = { onMouseDown: (e) => this.onDataPointClick?.(dataPoint, e) };

    const x = this.timeToPixel(dataPoint.timestamp);
    const commonOpts = {
      x,
      y: -height / 2 + width / 2,
      y2: height / 2 - width / 2,
      cursor: "pointer",
      cap: "round",
      eventHandlers,
    } as const;

    return Group.of({
      children: [
        // Using multiple layers, draw the data point with a dark border
        line({ ...commonOpts, width, color: dataPoint.color ?? defaultColor }),
        line({ ...commonOpts, width, color: colors.dataPointBorder }),
        line({ ...commonOpts, width: width - 2, color: dataPoint.color ?? defaultColor }),
      ],
    });
  });
}
