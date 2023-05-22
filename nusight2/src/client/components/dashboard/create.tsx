import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { DashboardController } from "./controller";
import { Field } from "./field/view";
import { DashboardModel } from "./model";
import { DashboardNetwork } from "./network";
import { Dashboard } from "./view";

export function createDashboardView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = DashboardModel.of(appModel.robots);
  const views = {
    Field: () => <Field model={model.field} />,
  };

  return () => {
    const network = DashboardNetwork.of(nusightNetwork);
    const controller = DashboardController.of();
    return <Dashboard controller={controller} Field={views.Field} Menu={Menu} model={model} network={network} />;
  };
}
