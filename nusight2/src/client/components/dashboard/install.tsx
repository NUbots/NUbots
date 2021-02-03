import React from 'react'
import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import { DashboardController } from './controller'
import { Field } from './field/view'
import Icon from './icon.svg'
import { DashboardModel } from './model'
import { DashboardNetwork } from './network'
import { Dashboard } from './view'

export function installDashboard({
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
  const model = DashboardModel.of(appModel.robots)
  const views = {
    Field: () => <Field model={model.field} />,
  }
  nav.addRoute({
    path: '/',
    exact: true,
    Icon,
    label: 'Dashboard',
    Content: () => {
      const network = DashboardNetwork.of(nusightNetwork)
      const controller = DashboardController.of()
      return (
        <Dashboard
          controller={controller}
          Field={views.Field}
          menu={menu}
          model={model}
          network={network}
        />
      )
    },
  })
}
