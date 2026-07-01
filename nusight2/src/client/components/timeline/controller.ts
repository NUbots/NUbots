import { action } from "mobx";

import { animateValues, AnimationGroup } from "./animation_group";
import { TimelineModel } from "./model";

export type TimelineEventHandler = (timestamp: bigint) => void;

export interface TimelineEventHandlers {
  onViewRangeChange?: TimelineEventHandler;
  onViewCentreChange?: TimelineEventHandler;
}

export class TimelineController {
  private viewAnimation?: AnimationGroup<[bigint, bigint]>;

  constructor(
    private model: TimelineModel,
    public handlers: TimelineEventHandlers,
  ) {}

  /**
   * Zoom the camera view in or out by a single step.
   * The position of the focal timestamp in the view will be maintained before/after the
   * zoom change.
   */
  @action.bound
  public zoom(direction: "in" | "out", focalTimestamp: bigint) {
    // If currently in an animation, move from the end values to not reset the view position
    const currentCentre = this.viewAnimation ? this.viewAnimation.endValues[0] : this.model.view.centre;
    const currentRange = this.viewAnimation ? this.viewAnimation.endValues[1] : this.model.view.range;
    this.viewAnimation?.cancel();

    // The percentage along the view to focus the zoom on
    const focalPercent = Number(focalTimestamp - (currentCentre - currentRange / 2n)) / Number(currentRange);

    // Zoom in or out depending on the direction
    const startRange = this.model.view.range;
    const divisor = 5n;

    // Since we are using bigints, we cannot use a multiplier to zoom in and out (due to integer math).
    // So we use addition instead by adding/subtracting a percentage of the current range.
    this.model.view.range =
      currentRange + (direction === "in" ? -currentRange / (divisor + 1n) : currentRange / divisor);

    // Emit the range, allowing it to be clamped if necessary
    this.handlers.onViewRangeChange?.(this.model.view.range);

    // Set the view center so that the focal timestamp is in the same position as before the zoom
    const startCentre = this.model.view.centre;
    const relativeFocalTimestamp = BigInt(Math.floor(focalPercent * Number(this.model.view.range)));
    const viewLeft = focalTimestamp - relativeFocalTimestamp;
    this.model.view.centre = viewLeft + this.model.view.range / 2n;

    // Emit the centre, allowing it to be clamped if necessary
    this.handlers.onViewCentreChange?.(this.model.view.centre);

    // Animate from the current view to the new view
    this.viewAnimation = animateValues({
      duration: 100,
      startValues: [startCentre, startRange],
      endValues: [this.model.view.centre, this.model.view.range],
      onUpdate: action(([centre, range]) => {
        this.model.view.centre = centre;
        this.model.view.range = range;
      }),
      onEnd: () => (this.viewAnimation = undefined),
    });
  }

  @action.bound
  public setViewCentre(centre: bigint) {
    this.model.view.centre = centre;
    this.handlers.onViewCentreChange?.(this.model.view.centre);
  }
}
