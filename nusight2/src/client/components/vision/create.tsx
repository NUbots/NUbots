import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { VisionController } from "./controller";
import { VisionModel } from "./model";
import { VisionNetwork } from "./network";
import { VisionView } from "./view";

export function createVisionView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = VisionModel.of(appModel);
  return () => {
    React.useEffect(() => VisionNetwork.of(nusightNetwork).destroy);
    const controller = VisionController.of();
    return <VisionView controller={controller} model={model} Menu={Menu} />;
  };
}
