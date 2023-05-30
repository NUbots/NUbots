import React, { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import IconChart from "./icon";

export function installChart({
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
    path: "/chart",
    Icon: IconChart,
    label: "Chart",
    Content: React.lazy(async () => {
      const { createChartView } = await import("./create");
      return {
        default: createChartView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
