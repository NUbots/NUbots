import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable, reaction } from 'mobx'
import { disposeOnUnmount, observer } from 'mobx-react'
import { now } from 'mobx-utils'
import React from 'react'

import { SeededRandom } from '../../../../shared/base/random/seeded_random'
import { RobotNetworkStatsModel } from '../../../network/model'
import { RobotModel } from '../../robot/model'
import { RobotSelector } from '../view'

const actions = {
  selectRobot: action('selectRobot'),
}

storiesOf('components.robot_selector', module)
  .addDecorator(story => <div style={{ maxWidth: '320px' }}>{story()}</div>)
  .add('renders empty', () => {
    return <RobotSelector robots={[]} selectRobot={actions.selectRobot} />
  })
  .add('renders with robots', () => {
    const robots = getRobots()
    return <RobotSelector robots={robots} selectRobot={actions.selectRobot} />
  })
  .add('renders with updating stats', () => {
    const robots = getRobots()
    const model = observable({
      robots,
    })
    return <UpdatingStatsStory robots={model.robots} />
  })
  .add('interactive', () => {
    const robots = getRobots()
    const model = observable({
      robots,
    })
    const selectRobot = mobxAction((robot: RobotModel) => (robot.enabled = !robot.enabled))
    const Component = observer(() => (
      <RobotSelector robots={model.robots} selectRobot={selectRobot} />
    ))
    return <Component />
  })

function getRobots(): RobotModel[] {
  return [
    {
      id: '1',
      name: 'Virtual Robot 1',
      connected: true,
      enabled: true,
      address: '10.222.10.14',
      port: 57619,
    },
    {
      id: '2',
      name: 'Virtual Robot 2',
      connected: true,
      enabled: true,
      address: '10.222.10.19',
      port: 36261,
    },
    {
      id: '3',
      name: 'Virtual Robot 3',
      connected: false,
      enabled: true,
      address: '10.222.10.20',
      port: 67341,
    },
  ]
}

const random = SeededRandom.of('random-stats')

@observer
export class UpdatingStatsStory extends React.Component<{ robots: RobotModel[] }> {
  render() {
    const { robots } = this.props

    return <RobotSelector robots={robots} selectRobot={actions.selectRobot} />
  }

  @mobxAction
  updateStats = () => {
    this.props.robots
      .filter(robotModel => robotModel.connected)
      .forEach(robotModel => {
        const stats = RobotNetworkStatsModel.of(robotModel)
        const packets = random.integer(1, 3)
        stats.packets += packets
        stats.packetsPerSecond.update(packets)
        const bytes = random.integer(1, 2e3)
        stats.bytes += bytes
        stats.bytesPerSecond.update(bytes)
      })
  }

  componentDidMount() {
    disposeOnUnmount(
      this,
      reaction(() => now('frame'), this.updateStats, { fireImmediately: true }),
    )
  }
}
