import React, { ComponentType } from 'react'
import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { AppModel } from '../app/model'
import { ReactionController } from './controller'
import Icon from './icon.svg'
import { ReactionModel } from './model'
import { ReactionNetwork } from './network'
import { ReactionView } from './view'

export function installReactions({
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
  const model = ReactionModel.of(appModel)
  nav.addRoute({
    path: '/reactions',
    Icon,
    label: 'Reaction Stats',
    Content: () => {
      React.useEffect(() => ReactionNetwork.of(nusightNetwork).destroy)
      const controller = ReactionController.of()
      return <ReactionView controller={controller} model={model} menu={Menu} />
    },
  })
}
