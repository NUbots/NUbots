import { Component } from 'react'
import * as React from 'react'
import { hot } from 'react-hot-loader'
import { Switch } from 'react-router'
import { Route } from 'react-router'
import { BrowserRouter } from 'react-router-dom'

import { NavigationConfiguration } from '../../navigation'
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
                {...this.nav.getRoutes().map(config => (
                  <Route key={config.path} exact={config.exact} path={config.path} render={() => <config.Content/>}/>
                ))}
              </Switch>
            </div>
          </div>
        </div>
      </BrowserRouter>
    )
  }
}


export default hot(module)(AppView)
