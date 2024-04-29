import React from "react";
import classNames from "classnames";
import { observer } from "mobx-react";

export type StatusIndicatorProps = {
  className?: string;
  connected: boolean;
};

export const StatusIndicator = observer((props: StatusIndicatorProps) => {
  const { connected, className } = props;
  const indicatorClassName = classNames("box-border w-3.5 h-3.5 rounded-md", className, {
    ["bg-green-200 shadow shadow-green-200"]: connected,
    ["bg-gray-300"]: !connected,
  });
  return <span className={indicatorClassName} title={connected ? "Connected" : "Disconnected"}></span>;
});
