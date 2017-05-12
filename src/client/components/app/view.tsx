import * as React from 'react'
import { NavigationView } from '../navigation/view'
import * as style from './style.css'

export class AppView extends React.Component<any, any> {
  public render() {
    return (
        <div className={style.app}>
          <NavigationView />
          <div className={style.app__container}>
            <div className={style.app__content}>
              {this.props.children}
              {this.renderDevTool()}
            </div>
          </div>
        </div>
    )
  }

  private renderDevTool() {
    if (process.env.NODE_ENV !== 'production') {
      const DevTools = require('mobx-react-devtools').default
      return <DevTools />
    }
  }
}
