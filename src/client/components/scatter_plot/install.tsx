import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { ScatterPlotView } from './view'

export function installScatterPlot({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/scatter',
    Icon,
    label: 'Scatter',
    Content: () => <ScatterPlotView/>,
  })
}
