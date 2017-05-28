import * as React from 'react'
import { HTMLProps } from 'react'
import * as style from './style.css'

export type MenuBarProps = HTMLProps<JSX.Element>

export const MenuBar = (props: MenuBarProps) => {
  return (
    <div className={style.menuBar}>
      {props.children}
    </div>
  )
}
