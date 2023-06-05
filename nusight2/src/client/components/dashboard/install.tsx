import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installDashboard({
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
    path: "/",
    Icon,
    label: "Dashboard",
    Content: React.lazy(async () => {
      const { createDashboardView } = await import("./create");
      return {
        default: createDashboardView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
