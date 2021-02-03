import classNames from 'classnames'
import React from 'react'
import { ChangeEvent } from 'react'
import { StatelessComponent } from 'react'

import CheckIcon from './check.svg'
import style from './style.css'

export interface CheckboxProps {
  checked: boolean
  disabled?: boolean

  onChange(event: ChangeEvent<HTMLInputElement>): void
}

export const Checkbox: StatelessComponent<CheckboxProps> = (props: CheckboxProps) => {
  const { checked, disabled, onChange } = props

  const backgroundClassName = classNames(style.background, {
    [style.checked]: checked,
    [style.disabled]: disabled,
  })

  const checkIconClassName = classNames(style.checkIcon, {
    [style.checkIconChecked]: checked,
  })

  return (
    <span className={style.checkbox}>
      <input
        type="checkbox"
        className={style.nativeControl}
        checked={checked}
        disabled={disabled}
        onChange={onChange}
      />
      <span className={backgroundClassName}>
        <CheckIcon className={checkIconClassName} />
      </span>
    </span>
  )
}
