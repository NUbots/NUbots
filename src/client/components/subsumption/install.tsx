import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { SubsumptionView } from './view'

export function installSubsumption({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/subsumption',
    Icon,
    label: 'Subsumption',
    Content: () => <SubsumptionView/>,
  })
}
