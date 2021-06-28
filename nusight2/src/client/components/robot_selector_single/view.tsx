import { computed } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import { RobotModel } from '../robot/model'
import { Option, Select } from '../select/view'

import PlugIcon from './plug.svg'
import RobotIcon from './robot.svg'
import style from './style.css'

export type RobotSelectorSingleProps = {
  robots: RobotModel[]
  selected?: RobotModel
  dropDirection?: 'up' | 'down'
  onSelect(robot?: RobotModel): void
}

@observer
export class RobotSelectorSingle extends React.Component<RobotSelectorSingleProps> {
  render() {
    const { dropDirection } = this.props
    return (
      <div className={style.robotSelector}>
        <Select
          options={this.options}
          selectedOption={this.selectedOption}
          onChange={this.onChange}
          placeholder="Select a robot..."
          empty={this.renderEmpty}
          icon={<RobotIcon />}
          dropDirection={dropDirection}
        />
      </div>
    )
  }

  @computed
  private get renderEmpty() {
    return (
      <div className={style.empty}>
        <div className={style.emptyIcon}>
          <PlugIcon />
        </div>
        <div className={style.emptyTitle}>No connected robots</div>
        <span className={style.emptyDescription}>Run yarn start:sim to simulate robots</span>
      </div>
    )
  }

  @computed
  private get options() {
    return this.props.robots.map(robot => ({
      id: robot.id,
      label: robot.name,
      robot,
    }))
  }

  @computed
  private get selectedOption() {
    if (this.props.selected) {
      return {
        id: this.props.selected.id,
        label: this.props.selected.name,
      }
    }
  }

  private onChange = (option: Option) => {
    this.props.onSelect(this.props.robots.find(robot => robot.id === option.id)!)
  }
}
