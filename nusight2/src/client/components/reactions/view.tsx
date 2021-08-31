import { action } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { RobotModel } from '../robot/model'
import { RobotSelectorSingle } from '../robot_selector_single/view'
import { ReactionController } from './controller'
import { ReactionModel } from './model'
import { ReactionVisualizer } from './odometry_visualizer/view'
import styles from './styles.css'

@observer
export class ReactionView extends React.Component<{
  model: ReactionModel
  menu: React.ComponentType
  controller: ReactionController
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      menu,
    } = this.props
    console.log({ selectedRobot, lastReaction: selectedRobot?.lastReaction })
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
        {selectedRobot && selectedRobot.lastReaction ? (
          <div className={styles.content}>
            <ReactionVisualizer stats={selectedRobot.lastReaction} />
            {/*<ReactionVisualizer stats={selectedRobot.visualizerModel} />*/}
          </div>
        ) : (
          <div>No robot selected or no reactions</div>
        )}
      </div>
    )
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot)
  }
}
