import { computed, observable } from "mobx";

import { clamp } from "../../../shared/math/clamp";
import { NbsScrubber } from "../../../shared/nbs_scrubber";
import { TimestampObject } from "../../../shared/time/timestamp";
import { RobotModel } from "../robot/model";

import { timestampToPercentage } from "./util";

export class NbsScrubbersModel {
  @observable scrubbers = new Map<NbsScrubber["id"], NbsScrubberModel>();
  @observable.ref lastActiveScrubber?: NbsScrubberModel;

  constructor(scrubbers: NbsScrubberModel[]) {
    scrubbers.forEach((scrubber) => this.scrubbers.set(scrubber.id, scrubber));
  }

  static of(opts?: { scrubbers?: NbsScrubberModel[] }) {
    return new NbsScrubbersModel(opts?.scrubbers ?? []);
  }

  /** Get the scrubber associated with the given robot model if any */
  scrubberOf(robot: RobotModel) {
    if (robot.type !== "nbs-scrubber") {
      return undefined;
    }

    return this.scrubbers.get(robot.port);
  }
}

export class NbsScrubberModel {
  @observable id: NbsScrubber["id"];
  @observable name: NbsScrubber["name"];
  @observable startTs: NbsScrubber["start"];
  @observable endTs: NbsScrubber["end"];
  @observable playbackState: NbsScrubber["playbackState"] = "paused";
  @observable playbackSpeed: NbsScrubber["playbackSpeed"] = 0;
  @observable playbackRepeat: NbsScrubber["playbackRepeat"] = false;

  @observable current = 0n;

  /** This indicates if this scrubber is currently waiting for a seek response from the network */
  @observable isSeeking = false;

  constructor(opts: NbsScrubber) {
    this.id = opts.id;
    this.name = opts.name;
    this.startTs = opts.start;
    this.endTs = opts.end;
    this.playbackState = opts.playbackState;
    this.playbackSpeed = opts.playbackSpeed;
    this.playbackRepeat = opts.playbackRepeat;
  }

  @computed
  get start(): bigint {
    return TimestampObject.toNanos(this.startTs);
  }

  @computed
  get end(): bigint {
    return TimestampObject.toNanos(this.endTs);
  }

  @computed
  get percentPlayed(): number {
    return clamp(timestampToPercentage(this.current, this.start, this.end), 0, 1);
  }

  @computed
  get duration(): bigint {
    return this.end - this.start;
  }

  @computed
  get startFormatted(): string {
    return TimestampObject.formatDate(this.start);
  }

  @computed
  get endFormatted(): string {
    return TimestampObject.formatDate(this.end);
  }

  @computed
  get durationFormatted(): string {
    return TimestampObject.formatDuration(this.duration);
  }

  @computed
  get currentFormatted(): string {
    return TimestampObject.formatDate(this.current);
  }

  @computed
  get playbackSpeedModifier(): number {
    return Math.pow(2, this.playbackSpeed);
  }
}
