import * as classNames from 'classnames'
import * as React from 'react'
import { ReactNode } from 'react'
import { MouseEvent } from 'react'
import { StatelessComponent } from 'react'
import * as style from './style.css'

export interface DropdownProps {
  children: ReactNode
  dropdownMenuClassName?: string
  dropdownToggle: ReactNode
  isOpen: boolean
  onRef(dropdown: HTMLDivElement): void
  onToggleClick?(event: MouseEvent<HTMLSpanElement>): void
}

export const Dropdown: StatelessComponent<DropdownProps> = (props: DropdownProps) => {
  const dropdownMenuClassName = classNames(style.dropdownMenu, props.dropdownMenuClassName)
  return (
    <div className={style.dropdown} ref={props.onRef}>
      <span className={style.dropdownToggle}
            onClick={props.onToggleClick}>
        {props.dropdownToggle}
      </span>
      {props.isOpen &&
        <div className={dropdownMenuClassName}>
          {props.children}
        </div>
      }
    </div>
  )
}
