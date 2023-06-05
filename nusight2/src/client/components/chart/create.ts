import { ComponentType } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { AppModel } from "../app/model";

import { LineChart } from "./line_chart/view";
import { ChartModel } from "./model";
import { ChartView } from "./view";

export function createChartView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
  Menu: ComponentType;
}) {
  const model = ChartModel.of({ robotModels: appModel.robots });
  return ChartView.of({ model, Menu, nusightNetwork, LineChart: LineChart.of(model) });
}
