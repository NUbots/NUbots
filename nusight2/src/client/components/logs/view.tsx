import React, { ComponentType, PropsWithChildren, useEffect } from "react";
import { observer } from "mobx-react";

import { LogsController } from "./controller";
import { LogLevel, LogMessage, LogsModel, LogsRobotModel } from "./model";
import { LogsNetwork } from "./network";
import { Icon } from "../icon/view";
import { RobotSelectorSingle } from "../robot_selector_single/view";

export interface LogsViewProps {
  controller: LogsController;
  Menu: ComponentType<PropsWithChildren>;
  model: LogsModel;
  network: LogsNetwork;
}

export const LogsView = observer(function LogsView(props: LogsViewProps) {
  const { Menu, model, controller } = props;

  const { robots, selectedLogsRobot } = model;

  return (
    <div className="w-full flex flex-col">
      <Menu>
        <div className="h-full flex items-center justify-end">
          <RobotSelectorSingle
            autoSelect
            robots={robots}
            selected={selectedLogsRobot?.robotModel}
            onSelect={controller.onSelectRobot}
          />
        </div>
      </Menu>
      {selectedLogsRobot ? (
        <div className="flex-grow border-t border-gray-300 flex flex-col">
          <div className="relative h-full w-full">
            <LogLines model={selectedLogsRobot} />
          </div>
        </div>
      ) : (
        <div className="flex flex-col justify-center items-center h-full w-full bg-gray-100">
          <div className="text-icon mb-2">
            <Icon size={48}>electrical_services</Icon>
          </div>
          <div className="text-2xl">No connected robots</div>
          <p className="text-sm opacity-80 mt-2">Connect a robot or load an NBS file to view logs</p>
        </div>
      )}
    </div>
  );
});

const logLevelToIcon: Record<LogLevel, string> = {
  unknown: "help",
  trace: "arrow_right_alt",
  debug: "bug_report",
  info: "info",
  warn: "warning",
  error: "error",
  fatal: "report",
};

interface LogLinesProps {
  model: LogsRobotModel;
}

const LogLines = observer(function LogLines(props: LogLinesProps) {
  const { model } = props;

  const scrollContainerRef = React.useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (scrollContainerRef.current) {
      scrollContainerRef.current.scrollTop = scrollContainerRef.current.scrollHeight;
    }
  });

  if (model.messages.length === 0) {
    return <div className="text-center py-4 text-lg text-gray-400">Waiting for log messages</div>;
  }

  return (
    <div ref={scrollContainerRef} className="absolute inset-0 overflow-auto font-mono min-h-0 overflow-y-auto">
      {model.messages.map((message, index) => (
        <LogLine key={index} message={message} />
      ))}
    </div>
  );
});

const logLevelToTextColor: Record<LogLevel, string> = {
  unknown: "text-gray-800",
  trace: "text-gray-800",
  debug: "text-green-800",
  info: "text-blue-800",
  warn: "text-yellow-800",
  error: "text-red-800",
  fatal: "text-red-800",
};

interface LogLineProps {
  message: LogMessage;
}

const LogLine = observer(function LogLine(props: LogLineProps) {
  const { message } = props;

  return (
    <div className={`flex gap-3 items-center py-0.5 border-b border-black/10 ${logLevelToTextColor[message.level]}`}>
      <div className="inline-flex items-end self-start gap-1">
        <div className="w-12 uppercase text-right">{message.level}</div>
        <Icon fill className="text-lg/none">
          {logLevelToIcon[message.level]}
        </Icon>
      </div>
      <div className="flex-grow whitespace-pre-wrap">{message.message}</div>
    </div>
  );
});
