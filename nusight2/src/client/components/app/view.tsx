import { Component } from 'react'
import React from 'react'
import { Switch } from 'react-router'
import { Route } from 'react-router'
import { BrowserRouter } from 'react-router-dom'

import { NavigationConfiguration } from '../../navigation'
import { NavigationView } from '../navigation/view'

import style from './style.css'

export class AppView extends Component<{ nav: NavigationConfiguration }> {
  render() {
    return (
      <BrowserRouter>
        <div className={style.app}>
          <NavigationView nav={this.props.nav} />
          <div className={style.app__container}>
            <div className={style.app__content}>
              <Switch>
                {...this.props.nav
                  .getRoutes()
                  .map(config => (
                    <Route
                      key={config.path}
                      exact={config.exact}
                      path={config.path}
                      component={config.Content}
                    />
                  ))}
              </Switch>
            </div>
          </div>
        </div>
      </BrowserRouter>
    )
  }
}
