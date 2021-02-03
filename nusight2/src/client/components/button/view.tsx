import classNames from 'classnames'
import React from 'react'
import { ReactNode } from 'react'

import styles from './styles.css'

export type ButtonProps = {
  type?: 'normal' | 'primary'
  fullwidth?: boolean
  textAlign?: 'left' | 'center' | 'right'
  disabled?: boolean
  iconBefore?: ReactNode
  iconAfter?: ReactNode
  iconAfterAlignedRight?: boolean
  children?: any
  onClick?(): void
}

export class Button extends React.PureComponent<ButtonProps> {
  render() {
    const {
      type = 'normal',
      textAlign = 'center',
      fullwidth,
      disabled,
      iconBefore,
      iconAfter,
      iconAfterAlignedRight,
      children,
      onClick,
    } = this.props
    return (
      <button
        onClick={onClick}
        disabled={disabled}
        className={classNames(styles.button, {
          [styles.buttonPrimary]: type === 'primary',
          [styles.buttonNormal]: type === 'normal',
          [styles.fullwidth]: fullwidth,
          [styles.iconAfterAlignedRight]: iconAfterAlignedRight,
          [styles.alignLeft]: textAlign === 'left',
          [styles.alignRight]: textAlign === 'right',
        })}
      >
        {iconBefore && <span className={styles.iconBefore}>{iconBefore}</span>}
        {children}
        {iconAfter && (
          <span
            className={classNames(styles.iconAfter, {
              [styles.iconAfterAlignedRight]: iconAfterAlignedRight,
            })}
          >
            {iconAfter}
          </span>
        )}
      </button>
    )
  }
}
