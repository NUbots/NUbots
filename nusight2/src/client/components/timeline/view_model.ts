import { Render2DEventHandlers } from "@client/render2d/event/event_handlers";
import { Vector2 } from "@shared/math/vector2";
import { action, computed, observable } from "mobx";

import { TimelineController } from "./controller";
import { TimelineModel } from "./model";

export interface Dimensions {
  width: number;
  height: number;
}

export abstract class TimelineViewModel {
  @observable.ref protected model: TimelineModel;
  @observable.ref protected controller: TimelineController;

  /** Current dimensions of the canvas */
  @observable public canvas: Dimensions = { width: 100, height: 100 };

  /** Current position of the mouse in the canvas */
  @observable public mousePos = Vector2.of();

  /** State of pan action */
  @observable private panningState: { startPos: number; startViewCentre: bigint } | null = null;

  constructor(model: TimelineModel, controller: TimelineController) {
    this.model = model;
    this.controller = controller;
  }

  @action
  setCanvasSize(size: Dimensions) {
    this.canvas = size;
  }

  /** Transform from a pixel x position on the canvas to a timestamp */
  pixelToTime(pixel: number) {
    const { range, centre } = this.model.view;
    const start = centre - range / 2n;
    const percentAcrossCanvas = pixel / this.canvas.width;
    return start + BigInt(Math.floor(Number(range) * percentAcrossCanvas));
  }

  /** Transform from a timestamp to a pixel x position on the canvas */
  timeToPixel(time: bigint): number {
    const { range, centre } = this.model.view;
    const start = centre - range / 2n;
    const percentAcrossCanvas = Number(time - start) / Number(range);
    return percentAcrossCanvas * this.canvas.width;
  }

  /** Transform from a timestamp to a pixel x position on the canvas */
  timeToCanvasPercent(time: bigint): number {
    const { range, centre } = this.model.view;
    const start = centre - range / 2n;
    const percentAcrossCanvas = Number(time - start) / Number(range);
    return percentAcrossCanvas;
  }

  /** The timestamp at the mouse position */
  @computed
  get hoveredTime(): bigint {
    return this.pixelToTime(this.mousePos.x);
  }

  /** Events that are triggered by mouse inputs on any part of the canvas */
  readonly defaultEventHandlers: Render2DEventHandlers = {
    /** Handle scrubbing and panning when the mouse is moved */
    onMouseMove: action((e) => {
      const mousePos = Vector2.of(e.x, e.y);
      if (this.panningState) {
        const mouseOffset = mousePos.x - this.panningState.startPos;
        const timeOffset = BigInt(Math.floor((mouseOffset / this.canvas.width) * Number(this.model.view.range)));
        this.controller.setViewCentre(this.panningState.startViewCentre - timeOffset);
      }
      this.mousePos = mousePos;
    }),

    /** Disable scrubbing and panning when the mouse is released */
    onMouseUp: action((e) => {
      if (e.button === 0) {
        this.panningState = null;
      }
    }),

    /** Start panning when the left mouse button is pressed */
    onMouseDown: action((e) => {
      if (e.button === 0) {
        this.panningState = { startPos: this.mousePos.x, startViewCentre: this.model.view.centre };
      }
    }),

    /** Zoom in or out when the scroll wheel is scrolled */
    onWheel: action((e) => {
      this.controller.zoom(Math.sign(e.deltaY) < 1 ? "in" : "out", this.pixelToTime(e.worldX));
    }),
  };

  /**
   * Evenly spaced timestamps in the current view range.
   *
   * Major grid lines are spaced timestamps where the distance between them is
   * the smallest power of 10 that is greater than a minimum spacing value, or half
   * that if it can fit.
   *
   * E.g. Major spaces go from 10s -> 5s -> 1s -> 0.5s -> 0.1s etc
   *
   * Minor grid lines are spaced by 1/10th of the major grid if it is a power of 10,
   * otherwise 1/5th of the major grid.
   */
  @computed
  get timeGridPositions() {
    const { min, max } = this.model.viewBounds;

    const minimumPixelSpacing = 200;
    const minimumSpacing = BigInt(
      Math.floor((minimumPixelSpacing / this.canvas.width) * Number(this.model.view.range)),
    );

    // Basically log10
    const power = BigInt(minimumSpacing.toString().length);

    let major = 10n ** power;
    const minor = major / 10n;

    if (major / 2n > minimumSpacing) {
      major /= 2n;
    }

    // Generate an array of steps from the given start to the given end, spaced by
    // the given step size
    function getSteps(start: bigint, end: bigint, step: bigint) {
      const length = (end - start) / step;
      return Array.from(Array(Number(length)).keys(), (i) => start + BigInt(i) * step);
    }

    // First major timestamp before start of view
    const start = min - (min % major);

    const minorSteps = getSteps(start, max + minor, minor)
      .filter((time) => time % major !== 0n)
      .map((time) => ({ time, position: this.timeToCanvasPercent(time) * this.canvas.width }));

    const majorSteps = getSteps(start, max + major, major).map((time) => ({
      time,
      position: this.timeToCanvasPercent(time) * this.canvas.width,
    }));

    return { major: majorSteps, minor: minorSteps };
  }
}
