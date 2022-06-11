import { ComponentType } from 'react'
import React from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import { KinematicsController } from './controller'
import Icon from './icon.svg'
import { KinematicsModel } from './model'
import { KinematicsNetwork } from './network'
import { KinematicsView } from './view'

export function installKinematics({
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
  const model = KinematicsModel.of(appModel)
  nav.addRoute({
    path: '/kinematics',
    Icon,
    label: 'Kinematics',
    Content: () => {
      React.useEffect(() => KinematicsNetwork.of(nusightNetwork, model).destroy)
      const controller = KinematicsController.of()
      return <KinematicsView controller={controller} Menu={Menu} model={model} />
    },
  })
}
