import { computed, observable } from "mobx";

import { NbsScrubber } from "../../../../../shared/nbs_scrubber";
import { percentageToTimestamp, timestampObjectToNanos } from "../util";

export class NbsScrubberModel {
  @observable id: NbsScrubber["id"];
  @observable name: NbsScrubber["name"];
  @observable startTs: NbsScrubber["start"];
  @observable endTs: NbsScrubber["end"];
  @observable playbackState: NbsScrubber["playbackState"] = "paused";
  @observable playbackSpeed: NbsScrubber["playbackSpeed"] = 0;
  @observable playbackRepeat: NbsScrubber["playbackRepeat"] = false;

  @observable percentPlayed = 0;
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
    return timestampObjectToNanos(this.startTs);
  }

  @computed
  get end(): bigint {
    return timestampObjectToNanos(this.endTs);
  }

  @computed
  get current(): bigint {
    return percentageToTimestamp(this.percentPlayed, this.start, this.end);
  }

  @computed
  get startFormatted(): string {
    return timestampToFormattedDate(this.start);
  }

  @computed
  get endFormatted(): string {
    return timestampToFormattedDate(this.end);
  }

  @computed
  get currentFormatted(): string {
    return timestampToFormattedDate(this.current);
  }

  @computed
  get playbackSpeedModifier(): number {
    return Math.pow(2, this.playbackSpeed);
  }
}

function timestampToFormattedDate(nanos: bigint): string {
  const date = new Date(Number(nanos / BigInt(1e6)));

  const d = String(date.getDate()).padStart(2, "0");
  const m = String(date.getMonth() + 1).padStart(2, "0");
  const y = String(date.getFullYear()).padStart(4, "0");

  const hr = String(date.getHours()).padStart(2, "0");
  const min = String(date.getMinutes()).padStart(2, "0");
  const sec = String(date.getSeconds()).padStart(2, "0");
  const ms = String(date.getMilliseconds()).padStart(3, "0");

  // Format in dd/mm/yyyy hh:mm:ss.mmm
  return `${d}/${m}/${y} ${hr}:${min}:${sec}.${ms}`;
}
