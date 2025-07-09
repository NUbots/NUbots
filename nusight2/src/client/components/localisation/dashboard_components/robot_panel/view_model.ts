import { computed } from "mobx";
import { createTransformer } from "mobx-utils";

import { Vector3 } from "../../../../../shared/math/vector3";
import { message } from "../../../../../shared/messages";
import { DashboardRobotModel } from "../dashboard_robot/model";

import { LastStatus } from "./view";

const Mode = message.input.GameState.Mode;
const PenaltyReason = message.input.GameState.PenaltyReason;
const Phase = message.input.GameState.Phase;

export class DashboardRobotPanelViewModel {
  constructor(private model: DashboardRobotModel) {}

  static of = createTransformer((model: DashboardRobotModel): DashboardRobotPanelViewModel => {
    return new DashboardRobotPanelViewModel(model);
  });

  @computed get connected(): boolean {
    return this.model.connected;
  }

  @computed
  get batteryValue(): string {
    const battery = this.model.battery;
    return battery === -1 ? "" : `${Math.round(battery * 100)}%`;
  }

  @computed
  get lastCameraImage(): LastStatus {
    return this.getLastStatus(this.model.lastCameraImage, 5);
  }

  @computed
  get lastSeenBall(): LastStatus {
    return this.getLastStatus(this.model.lastSeenBall, 30);
  }

  @computed
  get lastSeenGoal(): LastStatus {
    return this.getLastStatus(this.model.lastSeenGoal, 30);
  }

  @computed
  get mode(): string {
    return Mode[this.model.gameMode] || Mode[Mode.UNKNOWN_MODE];
  }

  @computed
  get penalised(): boolean {
    return this.model.penaltyReason !== PenaltyReason.UNPENALISED;
  }

  @computed
  get penalty(): string {
    return PenaltyReason[this.model.penaltyReason] || PenaltyReason[PenaltyReason.UNKNOWN_PENALTY_REASON];
  }

  @computed
  get phase(): string {
    return Phase[this.model.gamePhase] || Phase[Phase.UNKNOWN_PHASE];
  }

  @computed
  get title(): string {
    return this.model.name;
  }

  get walkCommand(): Vector3 {
    return this.model.walkCommand;
  }

  private getLastStatus(time: number, threshold: number): LastStatus {
    const value = (this.model.time - time) / threshold;
    return value < 0.5 ? "okay" : value > 0.9 ? "danger" : "warning";
  }
}
