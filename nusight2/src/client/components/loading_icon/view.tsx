import React from "react";

import style from "./style.module.css";

export class LoadingIcon extends React.PureComponent<{ size?: number }> {
  render() {
    const { size = 32 } = this.props;
    return (
      <svg width={size} height={size} className={style.loading} role="progressbar" viewBox="25 25 50 50">
        <circle className={style.circle} cx="50" cy="50" fill="none" r="20" strokeMiterlimit="10" strokeWidth="4" />
      </svg>
    );
  }
}
