import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installOdometry({
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
    path: "/odometry",
    Icon,
    label: "Odometry",
    Content: React.lazy(async () => {
      const { createOdometryView } = await import("./create");
      return {
        default: createOdometryView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
