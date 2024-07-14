import { computed, observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

export const ServoNames: { [key: number]: string } = {
  0: "R_SHOULDER_PITCH",
  1: "L_SHOULDER_PITCH",
  2: "R_SHOULDER_ROLL",
  3: "L_SHOULDER_ROLL",
  4: "R_ELBOW",
  5: "L_ELBOW",
  6: "R_HIP_YAW",
  7: "L_HIP_YAW",
  8: "R_HIP_ROLL",
  9: "L_HIP_ROLL",
  10: "R_HIP_PITCH",
  11: "L_HIP_PITCH",
  12: "R_KNEE",
  13: "L_KNEE",
  14: "R_ANKLE_PITCH",
  15: "L_ANKLE_PITCH",
  16: "R_ANKLE_ROLL",
  17: "L_ANKLE_ROLL",
  18: "HEAD_YAW",
  19: "HEAD_PITCH",
};

export class TempMonitorModel {
  @observable.ref selectedRobot?: TempMonitorRobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new TempMonitorModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.enabled);
  }
}

export class TempMonitorRobotModel {
  robotModel: RobotModel;
  @observable servoTemperatures: Map<number, number> = new Map();

  constructor(robotModel: RobotModel) {
    this.robotModel = robotModel;
  }

  static of = memoize((robotModel: RobotModel) => {
    return new TempMonitorRobotModel(robotModel);
  });

  @computed
  get averageTemperature(): number {
    const temps = Array.from(this.servoTemperatures.values());
    return temps.length > 0 ? temps.reduce((a, b) => a + b) / temps.length : 0;
  }

  @computed
  get highestTemperature(): number {
    return Math.max(...Array.from(this.servoTemperatures.values()), 0);
  }

  @computed
  get highestTemperatureServo(): { id: number; name: string; temperature: number } | null {
    if (this.servoTemperatures.size === 0) return null;
    const entries = Array.from(this.servoTemperatures.entries());
    const [id, temp] = entries.reduce((max, current) => (current[1] > max[1] ? current : max));
    return {
      id,
      name: ServoNames[id] || `Servo ${id}`,
      temperature: temp,
    };
  }
}
