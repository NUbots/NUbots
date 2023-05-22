import { ComponentType } from "react";
import React from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import Icon from "./icon";

export function installLocalisation({
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
    path: "/localisation",
    Icon,
    label: "Localisation",
    Content: React.lazy(async () => {
      const { createLocalisationView } = await import("./create");
      return {
        default: createLocalisationView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
