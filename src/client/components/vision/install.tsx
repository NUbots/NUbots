import React from 'react'
import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'
import { CameraView } from './camera/view'
import { VisionController } from './controller'

import Icon from './icon.svg'
import { VisionModel } from './model'
import { VisionNetwork } from './network'
import { VisionView } from './view'

export function installVision({
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
  const model = VisionModel.of(appModel)
  nav.addRoute({
    path: '/vision',
    Icon,
    label: 'Vision',
    Content: () => {
      React.useEffect(() => VisionNetwork.of(nusightNetwork).destroy)
      const controller = VisionController.of()
      return (
        <VisionView controller={controller} model={model} Menu={Menu} CameraView={CameraView} />
      )
    },
  })
}
