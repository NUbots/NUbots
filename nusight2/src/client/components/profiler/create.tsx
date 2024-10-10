import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { ProfilerController } from "./controller";
import { ProfilerModel } from "./model";
import { ProfilerNetwork } from "./network";
import { ProfilerView } from "./view";

export function createProfilerView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = ProfilerModel.of(appModel);
  return () => {
    React.useEffect(() => ProfilerNetwork.of(nusightNetwork).destroy);
    const controller = ProfilerController.of();
    return <ProfilerView controller={controller} model={model} Menu={Menu} />;
  };
}
