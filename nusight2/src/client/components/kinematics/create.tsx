import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { KinematicsController } from "./controller";
import { KinematicsModel } from "./model";
import { KinematicsNetwork } from "./network";
import { KinematicsView } from "./view";

export function createKinematicsView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = KinematicsModel.of(appModel);
  return () => {
    React.useEffect(() => KinematicsNetwork.of(nusightNetwork).destroy);
    const controller = KinematicsController.of();
    return <KinematicsView controller={controller} model={model} Menu={Menu} />;
  };
}
