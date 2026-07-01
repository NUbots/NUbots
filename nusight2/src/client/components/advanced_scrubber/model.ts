import { memoize } from "@client/base/memoize";
import { AppModel } from "@components/app/model";
import { NbsScrubberModel } from "@components/nbs_scrubbers/model";
import { RobotModel } from "@components/robot/model";
import { action, computed, observable } from "mobx";

export class AdvancedScrubberModel {
  @observable.ref selectedRobot?: RobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new AdvancedScrubberModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.type === "nbs-scrubber");
  }

  @action.bound
  setSelectedRobot(robot?: RobotModel) {
    this.selectedRobot = robot;
  }

  @computed
  get selectedScrubber(): AdvancedScrubberScrubberModel | undefined {
    if (this.selectedRobot) {
      const scrubber = this.appModel.scrubbersModel.scrubberOf(this.selectedRobot);
      return scrubber ? AdvancedScrubberScrubberModel.of(scrubber) : undefined;
    }
  }
}

export interface MessageTypeId {
  typeName: string;
  typeHash: string;
  subtype: number;
}

/** Scrubber timestamps of a message type subtype pair */
export interface TypeIndex {
  type: MessageTypeId;
  timestamps: bigint[];
}

export class AdvancedScrubberScrubberModel {
  /** Timestamps of each message type subtype of this model's scrubber */
  @observable.ref indices?: TypeIndex[];

  constructor(public scrubber: NbsScrubberModel) {}

  static of = memoize((scrubber: NbsScrubberModel) => {
    return new AdvancedScrubberScrubberModel(scrubber);
  });
}
