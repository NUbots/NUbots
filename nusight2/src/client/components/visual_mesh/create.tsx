import React from "react";
import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { VisualMeshModel } from "./model";
import { VisualMeshNetwork } from "./network";
import { VisualMeshView } from "./view";
import { VisualMeshViewModel } from "./view_model";

export function createVisualMeshView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = VisualMeshModel.of(appModel);
  return () => {
    const viewModel = VisualMeshViewModel.of(model);
    const network = VisualMeshNetwork.of(nusightNetwork);
    return <VisualMeshView viewModel={viewModel} network={network} Menu={Menu} />;
  };
}
