import { ComponentType } from 'react'

import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import { LineChart as LineChartImpl } from './line_chart/view'
import { ChartModel } from './model'
import { ChartView } from './view'

export function createChartView({
  appModel,
  nusightNetwork,
  Menu,
}: {
  appModel: AppModel
  nusightNetwork: NUsightNetwork
  Menu: ComponentType
}) {
  const model = ChartModel.of({ robotModels: appModel.robots })
  const LineChart = LineChartImpl.of(model)
  return ChartView.of({ model, menu: Menu, nusightNetwork, LineChart })
}
