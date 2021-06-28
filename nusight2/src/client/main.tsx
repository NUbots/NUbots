import { configure } from 'mobx'
import React from 'react'
import ReactDOM from 'react-dom'

import { AppController } from './components/app/controller'
import { AppModel } from './components/app/model'
import { AppNetwork } from './components/app/network'
import { AppView } from './components/app/view'
import { installChart } from './components/chart/install'
import { installDashboard } from './components/dashboard/install'
import { installLocalisation } from './components/localisation/install'
import { withRobotSelectorMenuBar } from './components/menu_bar/view'
import { installOdometry } from './components/odometry/install'
import { installVision } from './components/vision/install'
import { installVisualMesh } from './components/visual_mesh/install'
import { NavigationConfiguration } from './navigation'
import { NUsightNetwork } from './network/nusight_network'

const nav = NavigationConfiguration.of()
const appModel = AppModel.of()
const nusightNetwork = NUsightNetwork.of(appModel)
nusightNetwork.connect({ name: 'nusight' })

const appController = AppController.of()
AppNetwork.of(nusightNetwork, appModel)
const menu = withRobotSelectorMenuBar(appModel.robots, appController.toggleRobotEnabled)

installDashboard({ nav, appModel, nusightNetwork, menu })
installLocalisation({ nav, appModel, nusightNetwork, menu })
installOdometry({ nav, appModel, nusightNetwork, Menu: menu })
installChart({ nav, appModel, nusightNetwork, menu })
installVision({ nav, appModel, nusightNetwork, Menu: menu })
installVisualMesh({ nav, appModel, nusightNetwork, Menu: menu })

configure({ enforceActions: 'observed' })
ReactDOM.render(<AppView nav={nav} />, document.getElementById('root'))
