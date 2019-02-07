import { Component } from 'react'
import * as React from 'react'
import { hot } from 'react-hot-loader'
import { Switch } from 'react-router'
import { Route } from 'react-router'
import { BrowserRouter } from 'react-router-dom'

import { NavigationConfiguration } from '../../navigation'
import { Route as RouteConfig } from '../../navigation'
import { NavigationView } from '../navigation/view'

import { installNav } from './install'
import * as style from './style.css'

class AppView extends Component {
  private readonly nav: NavigationConfiguration = installNav()

  render() {
    return (
      <BrowserRouter>
        <div className={style.app}>
          <NavigationView nav={this.nav}/>
          <div className={style.app__container}>
            <div className={style.app__content}>
              <Switch>
                {...this.nav.getRoutes().map(config => <RouteView key={config.path} config={config}/>)}
              </Switch>
            </div>
          </div>
        </div>
      </BrowserRouter>
    )
  }
}

class RouteView extends Component<{ config: RouteConfig }> {
  render() {
    const { config } = this.props
    return <Route exact={config.exact} path={config.path} render={this.renderRoute}/>
  }

  private readonly renderRoute = () => {
    return <this.props.config.Content />
  }
}

export default hot(module)(AppView)
