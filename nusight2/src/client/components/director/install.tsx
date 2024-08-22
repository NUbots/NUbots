import React from "react";
import { ComponentType } from "react";

import { NavigationConfiguration } from "../../navigation";
import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";
import { Icon } from "../icon/view";

export function installDirector({
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
    path: "/director",
    Icon: () => <Icon>account_tree</Icon>,
    label: "Director",
    Content: React.lazy(async () => {
      const { createDirectorView } = await import("./create");
      return {
        default: createDirectorView({ appModel, nusightNetwork, Menu }),
      };
    }),
  });
}
