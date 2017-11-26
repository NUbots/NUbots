import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { NUClearView } from './view'

export function installNUClear({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/nuclear',
    Icon,
    label: 'NUClear',
    Content: () => <NUClearView/>,
  })
}
