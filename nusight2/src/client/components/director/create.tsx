import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { DirectorController } from "./controller";
import { DirectorModel } from "./model";
import { DirectorNetwork } from "./network";
import { DirectorView } from "./view";

export function createDirectorView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = DirectorModel.of(appModel);
  return () => {
    const network = DirectorNetwork.of(nusightNetwork);
    const controller = DirectorController.of(model);
    return <DirectorView controller={controller} model={model} Menu={Menu} network={network} />;
  };
}
