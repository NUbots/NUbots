import { LogLevelEnum } from "@proto/message/nuclear/LogLevel";
import { LogMessage } from "@proto/message/nuclear/LogMessage";
import { action } from "mobx";

import { Timestamp } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { LogLevel, LogsRobotModel } from "./model";

export class LogsNetwork {
  constructor(private network: Network) {
    this.network.on(LogMessage, this.onLogMessage);
  }

  static of(nusightNetwork: NUsightNetwork): LogsNetwork {
    const network = Network.of(nusightNetwork);
    return new LogsNetwork(network);
  }

  destroy() {
    this.network.off();
  }

  @action.bound
  private onLogMessage(robotModel: RobotModel, message: LogMessage) {
    const robot = LogsRobotModel.of(robotModel);

    robot.messages.push({
      level: nuclearLogLevelToLogLevel(message.level),
      timestamp: new Timestamp(message.timestamp!).toDate(),
      message: message.message,
      reactor: message.reactionStatistics!.identifiers!.reactor!,
    });
  }
}

/** Convert the given NUClear log level to a NUsight log level. */
function nuclearLogLevelToLogLevel(level: LogLevelEnum): LogLevel {
  switch (level) {
    case LogLevelEnum.TRACE:
      return "trace";
    case LogLevelEnum.DEBUG:
      return "debug";
    case LogLevelEnum.INFO:
      return "info";
    case LogLevelEnum.WARN:
      return "warn";
    case LogLevelEnum.ERROR:
      return "error";
    case LogLevelEnum.FATAL:
      return "fatal";
    default:
      return "unknown";
  }
}
