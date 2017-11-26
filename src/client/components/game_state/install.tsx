import * as React from 'react'
import { NavigationConfiguration } from '../../navigation'
import Icon from './icon.svg'
import { GameStateView } from './view'

export function installGameState({ nav }: { nav: NavigationConfiguration }) {
  nav.addRoute({
    path: '/gamestate',
    Icon,
    label: 'GameState',
    Content: () => <GameStateView/>,
  })
}
