import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import { SwitchesMenu, SwitchesMenuOption } from '../view'

const actions = {
  toggle: action('toggle'),
}

storiesOf('components.switches_menu', module)
  .addDecorator(story => <div style={{ maxWidth: '350px' }}>{story()}</div>)
  .add('renders empty', () => {
    return <SwitchesMenu options={[]} />
  })
  .add('renders with options', () => {
    return <SwitchesMenu options={getOptions()} />
  })
  .add('dropdown right', () => {
    const style = { backgroundColor: '#eee', display: 'flex', justifyContent: 'flex-end' }
    return (
      <div style={style}>
        <SwitchesMenu options={getOptions()} dropdownMenuPosition="right" />
      </div>
    )
  })
  .add('interactive', () => {
    const model = observable({
      options: getOptions().map(({ label, enabled }: SwitchesMenuOption, i) => {
        return {
          label,
          enabled,
          toggle: mobxAction(() => {
            model.options[i].enabled = !model.options[i].enabled
          }),
        }
      }),
    })
    const Component = observer(() => <SwitchesMenu options={model.options} />)

    return <Component />
  })

function getOptions(): SwitchesMenuOption[] {
  return [
    { label: 'Lines', enabled: true, toggle: actions.toggle },
    { label: 'Balls', enabled: true, toggle: actions.toggle },
    { label: 'Goals', enabled: false, toggle: actions.toggle },
  ]
}
