import { NavigationConfiguration } from '../../navigation'
import { NUsightNetwork } from '../../network/nusight_network'
import { installChart } from '../chart/install'
import { installDashboard } from '../dashboard/install'
import { installLocalisation } from '../localisation/install'
import { withRobotSelectorMenuBar } from '../menu_bar/view'
import { installVision } from '../vision/install'
import { installVisualMesh } from '../visual_mesh/install'

import { AppController } from './controller'
import { AppModel } from './model'
import { AppNetwork } from './network'

export function installNav() {
  const nav = NavigationConfiguration.of()
  const appModel = AppModel.of()
  const nusightNetwork = NUsightNetwork.of(appModel)
  nusightNetwork.connect({ name: 'nusight' })

  const appController = AppController.of()
  AppNetwork.of(nusightNetwork, appModel)
  const menu = withRobotSelectorMenuBar(appModel.robots, appController.toggleRobotEnabled)

  installDashboard({ nav, appModel, nusightNetwork, menu })
  installLocalisation({ nav, appModel, nusightNetwork, menu })
  installChart({ nav, appModel, nusightNetwork, menu })
  installVision({ nav, appModel, nusightNetwork, Menu: menu })
  installVisualMesh({ nav, appModel, nusightNetwork, Menu: menu })

  return nav
}
