import { action } from "mobx";

import { message, google } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { LogLevel, LogsRobotModel } from "./model";

import LogMessage = message.nuclear.LogMessage;
import NUClearLogLevel = message.nuclear.LogLevel;

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
      message: message.message,
      reactor: message.reactionStatistics!.identifiers!.reactor!,
    });
  }
}

/** Convert the given NUClear log level to a NUsight log level. */
function nuclearLogLevelToLogLevel(level: NUClearLogLevel): LogLevel {
  switch (level - 1) {
    case NUClearLogLevel.TRACE:
      return "trace";
    case NUClearLogLevel.DEBUG:
      return "debug";
    case NUClearLogLevel.INFO:
      return "info";
    case NUClearLogLevel.WARN:
      return "warn";
    case NUClearLogLevel.ERROR:
      return "error";
    case NUClearLogLevel.FATAL:
      return "fatal";
    default:
      return "unknown";
  }
}

/** Convert the given protobuf timestamp to a Date object */
function protobufTimestampToDate(timestamp: google.protobuf.ITimestamp) {
  const seconds = timestamp.seconds as number;
  const nanos = timestamp.nanos as number;
  return new Date(seconds * 1000 + nanos / 1000000);
}
