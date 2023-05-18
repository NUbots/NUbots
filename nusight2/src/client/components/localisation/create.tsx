import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { LocalisationController } from "./controller";
import { LocalisationModel } from "./model";
import { LocalisationNetwork } from "./network";
import { LocalisationView } from "./view";

export function createLocalisationView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = LocalisationModel.of(appModel);
  return () => {
    const network = LocalisationNetwork.of(nusightNetwork, model);
    const controller = LocalisationController.of();
    return <LocalisationView controller={controller} Menu={Menu} model={model} network={network} />;
  };
}
