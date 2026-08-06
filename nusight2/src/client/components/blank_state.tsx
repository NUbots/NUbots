import React from "react";
import { useUrlSelectedDataSource } from "@hooks/use_auto_select_data_source";

import { Icon } from "./icon/view";

interface BlankStateProps {
  title: string;
  icon?: React.ReactNode;
  children?: React.ReactNode;
}

export function BlankState({ icon, title, children }: BlankStateProps) {
  return (
    <div className="flex flex-col justify-center items-center h-full p-8">
      {icon ? (
        <div className="opacity-60 mb-2">
          {typeof icon === "string" ? <Icon className="text-6xl/none">{icon}</Icon> : icon}
        </div>
      ) : null}
      <div className="text-3xl font-light">{title}</div>
      {children ? <p className="text-base opacity-80 mt-4">{children}</p> : null}
    </div>
  );
}

interface NoSelectedDataSourceProps {
  /** Whether any robots are connected */
  hasDataSources: boolean;
  /** Whether a robot is required to be specified in the URL */
  urlSelectionRequired?: boolean;
  /** The action that the user would be performing if a robot was selected, shown in the message */
  pendingAction: string;
}

/**
 * Shows a blank state when no robot is connected or selected, with an appropriate message based on:
 * - whether a particular robot is specified for selection in the URL, and whether URL selection is required
 * - whether any robots are connected
 */
export function NoSelectedDataSource(props: NoSelectedDataSourceProps) {
  const { hasDataSources, pendingAction, urlSelectionRequired } = props;

  const urlDataSource = useUrlSelectedDataSource();

  // If a specific robot is expected for selection (i.e. there's a robot ID or name specified in the URL),
  // show a message indicating that
  if (urlDataSource) {
    return (
      <BlankState title="Waiting for robot to connect" icon="electrical_services">
        The specified robot <span className="font-bold">{urlDataSource.id ?? urlDataSource.name}</span> is not connected
      </BlankState>
    );
  }

  // Some views require a robot to be specified in the URL.
  // If this is the case and no robot is specified, show a message indicating that.
  if (urlSelectionRequired && !urlDataSource) {
    return (
      <BlankState title="No robot specified" icon="error">
        Specify a robot in the URL to {pendingAction}
      </BlankState>
    );
  }

  // If there are robots connected, show a message prompting the user to select one
  if (hasDataSources) {
    return (
      <BlankState title="No robot selected" icon="electrical_services">
        Select a robot to {pendingAction}
      </BlankState>
    );
  }

  // Otherwise, show a message prompting the user to connect a robot
  return (
    <BlankState title="No connected robots" icon="electrical_services">
      Connect a device or load an NBS file to {pendingAction}
    </BlankState>
  );
}
