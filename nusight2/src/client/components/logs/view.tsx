import React, { ComponentType, PropsWithChildren, useEffect } from "react";
import { observer } from "mobx-react";

import { Icon } from "../icon/view";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { LogsController } from "./controller";
import { LogLevel, LogMessage, LogsModel, LogsRobotModel } from "./model";
import { LogsNetwork } from "./network";

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
        <div className="flex-grow flex flex-col">
          <Toolbar model={selectedLogsRobot} controller={controller} />
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

interface ToolbarProps {
  model: LogsRobotModel;
  controller: LogsController;
}

const Toolbar = observer(function Toolbar(props: ToolbarProps) {
  const { model, controller } = props;

  return (
    <div className="bg-gray-100 dark:bg-gray-800 px-2 py-1.5 flex border-t border-b border-gray-300 dark:border-transparent">
      <SearchBox model={model} controller={controller} />

      <div className="flex gap-1 border-x border-gray-300 dark:border-transparent px-2 mx-2">
        <ToggleButton on={model.filters.levels.trace} onClick={(on) => controller.setFilter(model, "trace", !on)}>
          <Icon size={20}>{logLevelToIcon.trace}</Icon>
          Trace
        </ToggleButton>
        {console.log(model.filters.levels.trace)}
        <ToggleButton on={model.filters.levels.debug} onClick={(on) => controller.setFilter(model, "debug", !on)}>
          <Icon size={20}>{logLevelToIcon.debug}</Icon>
          Debug
        </ToggleButton>
        <ToggleButton on={model.filters.levels.info} onClick={(on) => controller.setFilter(model, "info", !on)}>
          <Icon size={20}>{logLevelToIcon.info}</Icon>
          Info
        </ToggleButton>
        <ToggleButton on={model.filters.levels.warn} onClick={(on) => controller.setFilter(model, "warn", !on)}>
          <Icon size={20}>{logLevelToIcon.warn}</Icon>
          Warn
        </ToggleButton>
        <ToggleButton on={model.filters.levels.error} onClick={(on) => controller.setFilter(model, "error", !on)}>
          <Icon size={20}>{logLevelToIcon.error}</Icon>
          Error
        </ToggleButton>
        <ToggleButton on={model.filters.levels.fatal} onClick={(on) => controller.setFilter(model, "fatal", !on)}>
          <Icon size={20}>{logLevelToIcon.fatal}</Icon>
          Fatal
        </ToggleButton>
      </div>

      <div>
        {/* {console.log(model.showTimestamps)} */}
        <ToggleButton on={model.showTimestamps} onClick={(on) => controller.setShowTimestamps(model, !on)}>
          <Icon size={20}>{model.showTimestamps ? "check_box" : "check_box_outline_blank"}</Icon>
          Show timestamps
        </ToggleButton>
      </div>
    </div>
  );
});

interface SearchBoxProps {
  model: LogsRobotModel;
  controller: LogsController;
}

const SearchBox = observer(function SearchBox(props: SearchBoxProps) {
  const { model, controller } = props;

  return (
    <div className="relative">
      <Icon className="absolute left-1 top-1 text-icon pointer-events-none">search</Icon>
      <input
        type="search"
        className="pl-8 pr-2 h-7 w-[320px] border border-gray-300 dark:border-gray-700 rounded bg-gray-200 dark:bg-gray-650 focus:outline-none focus:border-transparent focus:ring-2 focus:ring-gray-300 dark:focus:ring-gray-700"
        placeholder="Filter logs"
        value={model.filters.search}
        onChange={(e) => controller.setSearch(model, e.target.value)}
      />
    </div>
  );
});

interface ToggleButtonProps {
  on: boolean;
  children: React.ReactNode;
  onClick: (enabled: boolean) => void;
}

function ToggleButton(props: ToggleButtonProps) {
  return (
    <button
      className={`h-7 px-2 inline-flex items-center border rounded ${
        props.on
          ? "bg-blue-600 border-blue-700 text-gray-100"
          : "bg-gray-100 dark:bg-gray-650 border-gray-350 dark:border-gray-700"
      } `}
      onClick={() => props.onClick(props.on)}
    >
      {props.children}
    </button>
  );
}

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

  if (model.messagesFilteredBySearch.length === 0) {
    if (model.messages.length === 0) {
      return <div className="text-center py-4 text-lg text-gray-500 dark:text-gray-650">No log messages yet</div>;
    } else {
      return (
        <div className="text-center py-4 text-lg text-gray-500 dark:text-gray-650">
          No messages match your filters and search
        </div>
      );
    }
  }

  return (
    <div ref={scrollContainerRef} className="absolute inset-0 overflow-auto font-mono min-h-0 overflow-y-auto">
      {model.messagesFilteredBySearch.map((message, index) => (
        <LogLine key={index} model={model} message={message} />
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
  model: LogsRobotModel;
  message: LogMessage;
}

const LogLine = observer(function LogLine(props: LogLineProps) {
  const { model, message } = props;

  return (
    <div
      className={`flex gap - 3 items - center py - 0.5 border - b border - black / 10 ${
        logLevelToTextColor[message.level]
      } `}
    >
      <div className="inline-flex items-end self-start gap-1">
        <div className="w-12 uppercase text-right">{message.level}</div>
        <Icon fill className="text-lg/none">
          {logLevelToIcon[message.level]}
        </Icon>
      </div>
      {model.showTimestamps ? <div className="text-gray-500">{message.timestamp.toLocaleTimeString()}</div> : null}
      <div>{message.reactor.split("::").at(-1)}</div>
      <div className="flex-grow whitespace-pre-wrap">{message.message}</div>
    </div>
  );
});
