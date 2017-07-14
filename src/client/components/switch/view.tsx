import * as classNames from 'classnames'
import * as React from 'react'
import { ChangeEvent } from 'react'
import { StatelessComponent } from 'react'
import * as style from './style.css'

export interface SwitchProps {
  on: boolean
  disabled?: boolean
  onChange(event: ChangeEvent<HTMLInputElement>): void
}

export const Switch: StatelessComponent<SwitchProps> = (props: SwitchProps) => {
  const { disabled, on } = props
  const trackClassName = classNames(style.track, {
    [style.trackOff]: !on,
    [style.trackOn]: on
  })
  const thumbClassName = classNames(style.thumb, {
    [style.thumbOff]: !on,
    [style.thumbOn]: on
  })
  return (
    <span className={style.switch}>
      <input type='checkbox'
        checked={on}
        disabled={disabled}
        className={style.nativeControl}
        onChange={props.onChange} />
      <span className={trackClassName} />
      <span role='thumb' className={thumbClassName} />
    </span>
  )
}
