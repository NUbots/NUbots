import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import { RobotModel } from '../../robot/model'
import { RobotSelectorSingle } from '../view'

const actions = {
  onSelect: action('onSelect'),
}

storiesOf('components.robot_selector_single', module)
  .addDecorator(story => <div style={{ maxWidth: '320px' }}>{story()}</div>)
  .add('renders empty', () => {
    return <RobotSelectorSingle robots={[]} onSelect={actions.onSelect} />
  })
  .add('renders with robots', () => {
    const robots = getRobots()
    return <RobotSelectorSingle robots={robots} onSelect={actions.onSelect} />
  })
  .add('renders with selection', () => {
    const robots = getRobots()
    const selected = robots[0]
    return <RobotSelectorSingle robots={robots} selected={selected} onSelect={actions.onSelect} />
  })
  .add('interactive', () => {
    const robots = getRobots()
    const model = observable({
      robots,
      selected: robots[1],
    })
    const onSelect = mobxAction((robot: RobotModel) => (model.selected = robot))
    const Component = observer(() => (
      <RobotSelectorSingle robots={model.robots} selected={model.selected} onSelect={onSelect} />
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
      connected: true,
      enabled: true,
      address: '',
      port: 0,
    },
  ]
}
