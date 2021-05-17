import classNames from 'classnames'
import React from 'react'
import { ReactNode } from 'react'
import { MouseEvent } from 'react'
import { StatelessComponent } from 'react'

import style from './style.css'

export interface DropdownProps {
  children: ReactNode
  className?: string
  dropdownToggle: ReactNode
  isOpen: boolean
  isFullwidth?: boolean
  dropdownPosition?: 'left' | 'right'
  dropDirection?: 'up' | 'down'

  onRef?(dropdown: HTMLDivElement): void

  onToggleClick?(event: MouseEvent<HTMLSpanElement>): void
}

export const Dropdown: StatelessComponent<DropdownProps> = (props: DropdownProps) => {
  const fullwidth = props.isFullwidth ? style.dropdownMenuFullwidth : ''
  const position = props.dropdownPosition === 'right' ? style.dropdownMenuRight : ''
  const direction = props.dropDirection === 'up' ? style.dropdownMenuUp : style.dropdownMenuDown
  const dropdownMenuClassName = classNames(style.dropdownMenu, fullwidth, position, direction)

  return (
    <div className={classNames([style.dropdown, props.className])} ref={props.onRef}>
      <span className={style.dropdownToggle} onClick={props.onToggleClick}>
        {props.dropdownToggle}
      </span>
      {props.isOpen && <div className={dropdownMenuClassName}>{props.children}</div>}
    </div>
  )
}
