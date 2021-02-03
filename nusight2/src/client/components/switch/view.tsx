import classNames from 'classnames'
import React from 'react'
import { ChangeEvent } from 'react'

import style from './style.css'

export interface SwitchProps {
  on: boolean
  disabled?: boolean

  onChange(event: ChangeEvent<HTMLInputElement>): void
}

export const Switch = (props: SwitchProps) => {
  const { disabled, on } = props
  const trackClassName = classNames(style.track, {
    [style.trackOff]: !on,
    [style.trackOn]: on,
  })
  const thumbClassName = classNames(style.thumb, {
    [style.thumbOff]: !on,
    [style.thumbOn]: on,
  })
  return (
    <span className={style.switch}>
      <span className={trackClassName} />
      <span role="thumb" className={thumbClassName} />
      <input
        type="checkbox"
        checked={on}
        disabled={disabled}
        className={style.nativeControl}
        onChange={props.onChange}
      />
    </span>
  )
}
