import { action } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { RobotModel } from '../robot/model'
import { RobotSelectorSingle } from '../robot_selector_single/view'
import { OdometryController } from './controller'
import { OdometryModel } from './model'
import { OdometryNetwork } from './network'
import { OdometryVisualizer } from './odometry_visualizer/view'
import styles from './styles.css'

@observer
export class OdometryView extends React.Component<{
  model: OdometryModel
  menu: React.ComponentType
  controller: OdometryController
  network: OdometryNetwork
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      menu,
    } = this.props
    return (
      <div className={styles.odometry}>
        <menu>
          <div className={styles.selector}>
            <RobotSelectorSingle
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </menu>
        {selectedRobot && (
          <div className={styles.content}>
            <OdometryVisualizer
              name={"name"}
              trigger_name={"trigger_name"}
              function_name={"function_name"}
              reaction_id={0}
              task_id={0}
              cause_reaction_id={0}
              cause_task_id={0}
              emitted={0}
              started={0}
              finished={0}
            />
            {/*<OdometryVisualizer stats={selectedRobot.visualizerModel} />*/}
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
