import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import Icon from './icon.svg'
import { LineChart as LineChartImpl } from './line_chart/view'
import { ChartModel } from './model'
import { ChartView as ChartViewImpl } from './view'

export function installChart({
  nav,
  appModel,
  nusightNetwork,
  menu,
}: {
  nav: NavigationConfiguration
  appModel: AppModel
  nusightNetwork: NUsightNetwork
  menu: ComponentType
}) {
  const model = ChartModel.of({ robotModels: appModel.robots })
  const LineChart = LineChartImpl.of(model)
  const ChartView = ChartViewImpl.of({ model, menu, nusightNetwork, LineChart })
  nav.addRoute({ path: '/chart', exact: true, Icon, label: 'Chart', Content: ChartView })
}
