import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { LogsController } from "./controller";
import { LogsModel } from "./model";
import { LogsNetwork } from "./network";
import { LogsView } from "./view";

export function createLogsView(opts: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}): () => JSX.Element {
  const { appModel, nusightNetwork, Menu } = opts;

  const model = LogsModel.of(appModel);

  return () => {
    const network = LogsNetwork.of(nusightNetwork);
    const controller = LogsController.of(model);
    return <LogsView controller={controller} Menu={Menu} model={model} network={network} />;
  };
}
