import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installVisualMesh({
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
    path: "/visualmesh",
    Icon,
    label: "Visual Mesh",
    Content: React.lazy(async () => {
      const { createVisualMeshView } = await import("./create");
      return {
        default: createVisualMeshView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
