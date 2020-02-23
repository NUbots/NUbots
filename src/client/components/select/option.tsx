import classNames from 'classnames'
import { action } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import style from './style.css'
import { Option } from './view'

export type SelectOptionProps = {
  className?: string
  option: Option
  isSelected: boolean
  onSelect(option: Option): void
}

@observer
export class SelectOption extends React.Component<SelectOptionProps> {
  render(): JSX.Element {
    const { className, option, isSelected } = this.props

    return (
      <div
        className={classNames([className, style.option, isSelected ? style.optionSelected : ''])}
        onClick={this.onSelect}
      >
        {option.label}
      </div>
    )
  }

  @action.bound
  private onSelect() {
    this.props.onSelect(this.props.option)
  }
}
