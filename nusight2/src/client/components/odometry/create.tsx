import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { OdometryController } from "./controller";
import { OdometryModel } from "./model";
import { OdometryNetwork } from "./network";
import { OdometryView } from "./view";

export function createOdometryView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = OdometryModel.of(appModel);
  return () => {
    React.useEffect(() => OdometryNetwork.of(nusightNetwork).destroy);
    const controller = OdometryController.of();
    return <OdometryView controller={controller} model={model} Menu={Menu} />;
  };
}
