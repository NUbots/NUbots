import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installTempMonitor({
  nav,
  appModel,
  nusightNetwork,
  Menu,
}: {
  nav: NavigationConfiguration;
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  nav.addRoute({
    path: "/temperature_monitor",
    Icon,
    label: "Temperature Monitor",
    Content: React.lazy(async () => {
      const { createTempMonitorView } = await import("./create");
      return {
        default: createTempMonitorView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
