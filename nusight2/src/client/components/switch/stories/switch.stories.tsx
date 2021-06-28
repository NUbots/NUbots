import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import { Switch } from '../view'

storiesOf('component.switch', module)
  .add('Interactive', () => {
    const model = observable({ on: false })
    const onChange = mobxAction(() => (model.on = !model.on))
    const Component = observer(() => <Switch on={model.on} onChange={onChange} />)
    return <Component />
  })
  .add('on', () => {
    return <Switch on={true} onChange={action('onChange')} />
  })
  .add('off', () => {
    return <Switch on={false} onChange={action('onChange')} />
  })
