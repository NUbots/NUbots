import { action } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { RobotModel } from '../robot/model'
import { RobotSelectorSingle } from '../robot_selector_single/view'
import { OdometryController } from './controller'
import { OdometryModel } from './model'
import { OdometryVisualizer } from './odometry_visualizer/view'
import styles from './styles.css'

@observer
export class OdometryView extends React.Component<{
  controller: OdometryController
  model: OdometryModel
  Menu: React.ComponentType
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props
    return (
      <div className={styles.odometry}>
        <Menu>
          <div className={styles.selector}>
            <RobotSelectorSingle
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        {selectedRobot && (
          <div className={styles.content}>
            <OdometryVisualizer model={selectedRobot.visualizerModel} />
          </div>
        )}
      </div>
    )
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot)
  }
}
