import React from 'react'

import styles from './styles.css'

export class LoadingIcon extends React.PureComponent<{ size?: number }> {
  render() {
    const { size = 32 } = this.props
    return (
      <svg
        width={size}
        height={size}
        className={styles.loading}
        role="progressbar"
        viewBox="25 25 50 50"
      >
        <circle
          className={styles.circle}
          cx="50"
          cy="50"
          fill="none"
          r="20"
          strokeMiterlimit="10"
          strokeWidth="4"
        />
      </svg>
    )
  }
}
