import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable } from 'mobx'
import { observer } from 'mobx-react'
import * as React from 'react'

import { RobotModel } from '../../robot/model'
import { RobotSelector } from '../view'

const actions = {
  selectRobot: action('selectRobot'),
}

storiesOf('components.robot_selector', module)
  .addDecorator(story => <div style={{ maxWidth: '320px' }}>{story()}</div>)
  .add('renders empty', () => {
    return <RobotSelector
      robots={[]}
      selectRobot={actions.selectRobot}
    />
  })
  .add('renders with robots', () => {
    const robots = getRobots()
    return <RobotSelector
      robots={robots}
      selectRobot={actions.selectRobot}
    />
  })
  .add('interactive', () => {
    const robots = getRobots()
    const model = observable({
      robots,
    })
    const selectRobot = mobxAction((robot: RobotModel) => robot.enabled = !robot.enabled)
    const Component = observer(() => <RobotSelector
      robots={model.robots}
      selectRobot={selectRobot}
    />)
    return <Component/>
  })

function getRobots(): RobotModel[] {
  return [
    {
      id: '1',
      name: 'Virtual Robot 1',
      connected: true,
      enabled: true,
      address: '',
      port: 0,
    },
    {
      id: '2',
      name: 'Virtual Robot 2',
      connected: true,
      enabled: true,
      address: '',
      port: 0,
    },
    {
      id: '3',
      name: 'Virtual Robot 3',
      connected: false,
      enabled: true,
      address: '',
      port: 0,
    },
  ]
}
