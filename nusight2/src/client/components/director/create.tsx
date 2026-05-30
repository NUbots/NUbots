import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { DirectorController } from "./controller";
import { DirectorModel } from "./model";
import { DirectorNetwork } from "./network";
import { DirectorView } from "./view";

export function createDirectorView(opts: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}): () => JSX.Element {
  const { appModel, nusightNetwork, Menu } = opts;

  const model = DirectorModel.of(appModel);

  return () => {
    React.useEffect(() => DirectorNetwork.of(nusightNetwork, model).destroy, [model]);
    const controller = DirectorController.of();
    return <DirectorView controller={controller} model={model} Menu={Menu} />;
  };
}
