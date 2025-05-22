import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installProfiler({
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
    path: "/profiler",
    Icon,
    label: "Profiler",
    Content: React.lazy(async () => {
      const { createProfilerView } = await import("./create");
      return {
        default: createProfilerView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
