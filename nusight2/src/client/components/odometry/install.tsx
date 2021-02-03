import React from 'react'
import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'
import { OdometryController } from './controller'

import Icon from './icon.svg'
import { OdometryModel } from './model'
import { OdometryNetwork } from './network'
import { OdometryView } from './view'

export function installOdometry({
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
  const model = OdometryModel.of(appModel)
  nav.addRoute({
    path: '/odometry',
    Icon,
    label: 'Odometry',
    Content: () => {
      React.useEffect(() => OdometryNetwork.of(nusightNetwork).destroy)
      const controller = OdometryController.of()
      return <OdometryView controller={controller} model={model} Menu={Menu} />
    },
  })
}
