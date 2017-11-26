import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { ChartView } from './view'

export function installChart({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/chart',
    Icon,
    label: 'Chart',
    Content: () => <ChartView/>,
  })
}
