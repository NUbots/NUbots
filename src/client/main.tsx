import { useStrict } from 'mobx'
import * as React from 'react'
import * as ReactDOM from 'react-dom'
import { BrowserRouter } from 'react-router-dom'
import { Route } from 'react-router-dom'
import { Switch } from 'react-router-dom'

import { AppController } from './components/app/controller'
import { AppModel } from './components/app/model'
import { AppNetwork } from './components/app/network'
import { AppView } from './components/app/view'
import { installDashboard } from './components/dashboard/install'
import { installLocalisation } from './components/localisation/install'
import { withRobotSelectorMenuBar } from './components/menu_bar/view'
import { NavigationConfiguration } from './navigation'
import { NUsightNetwork } from './network/nusight_network'

// enable MobX strict mode
useStrict(true)

const appModel = AppModel.of()
const nusightNetwork = NUsightNetwork.of(appModel)
nusightNetwork.connect({ name: 'nusight' })

const appController = AppController.of()
AppNetwork.of(nusightNetwork, appModel)
const menu = withRobotSelectorMenuBar(appModel.robots, appController.toggleRobotEnabled)

const nav = NavigationConfiguration.of()
installDashboard({ nav, appModel, nusightNetwork, menu })
installLocalisation({ nav, appModel, nusightNetwork, menu })

ReactDOM.render(
  <BrowserRouter>
    <AppView nav={nav}>
      <Switch>
        {...nav.getRoutes().map(config => (
          <Route key={config.path} exact={config.exact} path={config.path} render={() => <config.Content/>}/>
        ))}
      </Switch>
    </AppView>
  </BrowserRouter>,
  document.getElementById('root'),
)

