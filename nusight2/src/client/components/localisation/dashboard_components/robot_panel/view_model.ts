import { computed } from "mobx";
import { createTransformer } from "mobx-utils";

import { Vector3 } from "../../../../../shared/math/vector3";
import { GameState_ModeEnum, GameState_PhaseEnum, GameState_PenaltyReasonEnum } from "@proto/message/input/GameState";
import { DashboardRobotModel } from "../dashboard_robot/model";

import { LastStatus } from "./view";

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
    return GameState_ModeEnum[this.model.gameMode] || GameState_ModeEnum[GameState_ModeEnum.UNKNOWN_MODE];
  }

  @computed
  get penalised(): boolean {
    return this.model.penaltyReason !== GameState_PenaltyReasonEnum.UNPENALISED;
  }

  @computed
  get penalty(): string {
    return GameState_PenaltyReasonEnum[this.model.penaltyReason] || GameState_PenaltyReasonEnum[GameState_PenaltyReasonEnum.UNPENALISED];
  }

  @computed
  get phase(): string {
    return GameState_PhaseEnum[this.model.gamePhase] || GameState_PhaseEnum[GameState_PhaseEnum.UNKNOWN_PHASE];
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
