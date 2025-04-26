import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installKinematics({
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
    path: "/kinematics",
    Icon,
    label: "Kinematics",
    Content: React.lazy(async () => {
      const { createKinematicsView } = await import("./create");
      return {
        default: createKinematicsView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
