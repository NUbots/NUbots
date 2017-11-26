import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { ClassifierView } from './view'

export function installClassifier({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/classifier',
    Icon,
    label: 'Classifier',
    Content: () => <ClassifierView/>,
  })
}
