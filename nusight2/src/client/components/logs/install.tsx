import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";
import { Icon } from "../icon/view";

export function installLogs(opts: {
  nav: NavigationConfiguration;
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const { nav, appModel, nusightNetwork, Menu } = opts;

  nav.addRoute({
    path: "/logs",
    Icon: () => <Icon>article</Icon>,
    label: "Logs",
    Content: React.lazy(async () => {
      const { createLogsView } = await import("./create");
      return {
        default: createLogsView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
