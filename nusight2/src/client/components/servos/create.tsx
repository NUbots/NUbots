import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { ServosController } from "./controller";
import { ServosModel } from "./model";
import { ServosNetwork } from "./network";
import { ServosView } from "./view";

export function createServosView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = ServosModel.of(appModel);
  return () => {
    React.useEffect(() => ServosNetwork.of(nusightNetwork).destroy);
    const controller = ServosController.of();
    return <ServosView controller={controller} model={model} Menu={Menu} />;
  };
}
