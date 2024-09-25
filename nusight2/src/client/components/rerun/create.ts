import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { RerunModel } from "./model";
import { RerunView } from "./view";

export function createRerunView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = RerunModel.of({ robotModels: appModel.robots });
  return RerunView.of({ model, Menu, nusightNetwork });
}
