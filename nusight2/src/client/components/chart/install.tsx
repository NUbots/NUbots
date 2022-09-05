import React from 'react'
import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import Icon from './icon.svg'

export function installChart({
  nav,
  appModel,
  nusightNetwork,
  Menu,
}: {
  nav: NavigationConfiguration
  appModel: AppModel
  nusightNetwork: NUsightNetwork
  Menu: ComponentType
}) {
  nav.addRoute({
    path: '/chart',
    Icon,
    label: 'Chart',
    Content: React.lazy(async () => {
      const { createChartView } = await import('./main')
      return {
        default: createChartView({ appModel, nusightNetwork, Menu }),
      }
    }),
  })
}
