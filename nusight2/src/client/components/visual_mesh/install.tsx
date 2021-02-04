import React from 'react'
import { ComponentType } from 'react'

import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'

import Icon from './icon.svg'
import { VisualMeshModel } from './model'
import { VisualMeshNetwork } from './network'
import { VisualMeshView } from './view'
import { VisualMeshViewModel } from './view_model'

export function installVisualMesh({
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
  const model = VisualMeshModel.of(appModel)
  nav.addRoute({
    path: '/visualmesh',
    Icon,
    label: 'Visual Mesh',
    Content: () => {
      const viewModel = VisualMeshViewModel.of(model)
      const network = VisualMeshNetwork.of(nusightNetwork)
      return <VisualMeshView viewModel={viewModel} network={network} Menu={Menu} />
    },
  })
}
