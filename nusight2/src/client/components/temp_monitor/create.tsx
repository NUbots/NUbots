import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { TempMonitorController } from "./controller";
import { TempMonitorModel } from "./model";
import { TempMonitorNetwork } from "./network";
import { TempMonitorView } from "./view";

export function createTempMonitorView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = TempMonitorModel.of(appModel);
  return () => {
    React.useEffect(() => TempMonitorNetwork.of(nusightNetwork).destroy);
    const controller = TempMonitorController.of();
    return <TempMonitorView controller={controller} model={model} Menu={Menu} />;
  };
}
