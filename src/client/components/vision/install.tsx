import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { VisionView } from './view'

export function installVision({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/vision',
    Icon,
    label: 'Vision',
    Content: () => <VisionView/>,
  })
}
