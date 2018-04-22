import * as React from 'react'
import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import Icon from './icon.svg'
import { VisionModel } from './model'
import { VisionNetwork } from './network'
import { VisionView } from './view'
import { VisionViewModel } from './view_model'

export function installVision({ nav, appModel, nusightNetwork, Menu }: {
  nav: NavigationConfiguration,
  appModel: AppModel,
  nusightNetwork: NUsightNetwork,
  Menu: ComponentType
}) {
  const model = VisionModel.of(appModel)
  nav.addRoute({
    path: '/vision',
    Icon,
    label: 'Vision',
    Content: () => {
      const viewModel = VisionViewModel.of(model)
      const network = VisionNetwork.of(nusightNetwork)
      return <VisionView viewModel={viewModel} network={network} Menu={Menu}/>
    },
  })
}
