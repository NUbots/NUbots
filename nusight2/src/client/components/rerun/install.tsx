import React, { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import IconRerun from "./icon";

export function installRerun({
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
    path: "/Rerun",
    Icon: IconRerun,
    label: "Rerun",
    Content: React.lazy(async () => {
      const { createRerunView } = await import("./create");
      return {
        default: createRerunView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
