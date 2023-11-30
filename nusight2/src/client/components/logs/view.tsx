import React, { ComponentType, PropsWithChildren } from "react";
import { observer } from "mobx-react";

import { LogsController } from "./controller";
import { LogsModel } from "./model";
import { LogsNetwork } from "./network";

export interface LogsViewProps {
  controller: LogsController;
  Menu: ComponentType<PropsWithChildren>;
  model: LogsModel;
  network: LogsNetwork;
}

export const LogsView = observer(function LogsView(props: LogsViewProps) {
  return <div>Logs Tab</div>;
});
