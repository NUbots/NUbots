import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installServos({
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
    path: "/servos",
    Icon,
    label: "Servos",
    Content: React.lazy(async () => {
      const { createServosView } = await import("./create");
      return {
        default: createServosView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
