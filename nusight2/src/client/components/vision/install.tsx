import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import IconEye from "./icon";

export function installVision({
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
    path: "/vision",
    Icon: IconEye,
    label: "Vision",
    Content: React.lazy(async () => {
      const { createVisionView } = await import("./create");
      return {
        default: createVisionView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
