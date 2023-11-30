import { computed, observable } from "mobx";

import { memoize } from "../../base/memoize";
import { RobotModel } from "../robot/model";
import { AppModel } from "../app/model";

/** The log levels that NUsight supports */
export type LogLevel = "unknown" | "trace" | "debug" | "info" | "warn" | "error" | "fatal";

export interface LogMessage {
  level: LogLevel;
  message: string;
  reactor: string;
}

export class LogsModel {
  private appModel: AppModel;

  @observable.ref selectedRobot?: RobotModel;

  constructor(appModel: AppModel) {
    this.appModel = appModel;
  }

  static of = memoize((appModel: AppModel) => {
    return new LogsModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((robot) => robot.enabled);
  }

  @computed
  get logsRobots(): LogsRobotModel[] {
    return this.robots.map(LogsRobotModel.of);
  }

  @computed
  get selectedLogsRobot(): LogsRobotModel | undefined {
    return this.selectedRobot ? LogsRobotModel.of(this.selectedRobot) : undefined;
  }
}

export interface FilterLevels {
  unknown: boolean;
  trace: boolean;
  debug: boolean;
  info: boolean;
  warn: boolean;
  error: boolean;
  fatal: boolean;
}

export class LogsRobotModel {
  robotModel: RobotModel;

  @observable.shallow messages: LogMessage[] = [];

  @observable filters: {
    search: string;
    levels: FilterLevels;
  };

  constructor(robotModel: RobotModel) {
    this.robotModel = robotModel;
    this.filters = {
      search: "",
      levels: {
        unknown: true,
        trace: false,
        debug: true,
        info: true,
        warn: true,
        error: true,
        fatal: true,
      },
    };
  }

  static of = memoize((robot: RobotModel): LogsRobotModel => {
    return new LogsRobotModel(robot);
  });

  @computed
  get messagesFilteredByLevel(): LogMessage[] {
    return this.messages.filter((message) => {
      return this.filters.levels[message.level];
    });
  }

  @computed
  get messagesFilteredBySearch(): LogMessage[] {
    const search = this.filters.search.toLowerCase();
    return this.messagesFilteredByLevel.filter((message) => {
      return message.message.toLowerCase().includes(search);
    });
  }
}
