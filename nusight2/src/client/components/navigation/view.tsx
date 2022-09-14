import classNames from 'classnames'
import React from 'react'
import { NavLink } from 'react-router-dom'

import { NavigationConfiguration } from '../../navigation'

import style from './style.module.css'

interface NavigationItemViewProps {
  url: string
  Icon: any
  children?: any
}

const NavigationItemView = ({ url, Icon, children }: NavigationItemViewProps) => (
  <li className={style.header__item}>
    <NavLink
      className={({ isActive }) =>
        classNames(style.header__link, {
          [style['header__link--active']]: isActive,
        })
      }
      to={url}
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
      {nav.getRoutes().map(config => (
        <NavigationItemView key={config.path} url={config.path} Icon={config.Icon}>
          {config.label}
        </NavigationItemView>
      ))}
    </ul>
  </header>
)
