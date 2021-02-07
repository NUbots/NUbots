import React from 'react'
import { NavLink } from 'react-router-dom'

import { NavigationConfiguration } from '../../navigation'

import style from './style.css'

interface NavigationItemViewProps {
  exact?: boolean
  url: string
  Icon: any
  children?: any
}

const NavigationItemView = ({ exact = false, url, Icon, children }: NavigationItemViewProps) => (
  <li className={style.header__item}>
    <NavLink
      exact={exact}
      className={style.header__link}
      to={url}
      activeClassName={style['header__link--active']}
    >
      <Icon className={style.header__icon} />
      <span>{children}</span>
    </NavLink>
  </li>
)

export const NavigationView = ({ nav }: { nav: NavigationConfiguration }) => (
  <header className={style.header}>
    <h1 className={style.header__title}>NUsight</h1>
    <ul className={style.header__list}>
      {...nav.getRoutes().map(config => (
        <NavigationItemView
          key={config.path}
          exact={config.exact}
          url={config.path}
          Icon={config.Icon}
        >
          {config.label}
        </NavigationItemView>
      ))}
    </ul>
  </header>
)
