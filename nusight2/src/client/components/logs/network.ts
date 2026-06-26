import { action } from "mobx";

import { LogMessage } from "@proto/message/nuclear/LogMessage";
import { LogLevelEnum } from "@proto/message/nuclear/LogLevel";
import { ITimestamp } from "@proto/google/protobuf/timestamp";
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
      timestamp: protobufTimestampToDate(message.timestamp!),
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

/** Convert the given protobuf timestamp to a Date object */
function protobufTimestampToDate(timestamp: ITimestamp) {
  const seconds = Number(timestamp.seconds ?? 0);
  const nanos = timestamp.nanos ?? 0;
  return new Date(seconds * 1000 + nanos / 1000000);
}
