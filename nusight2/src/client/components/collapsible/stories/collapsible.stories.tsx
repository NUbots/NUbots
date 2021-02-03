import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import { Collapsible } from '../view'

const actions = {
  onToggle: action('onToggle'),
}

storiesOf('components.collapsible', module)
  .addDecorator(story => <div style={{ maxWidth: '320px' }}>{story()}</div>)
  .add('basic', () => {
    return (
      <Collapsible open={true} onToggle={actions.onToggle}>
        <div>Collapsible content</div>
        <div>
          Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum,
          tenetur eveniet. Adipisci sed, labore eos molestias.
        </div>
      </Collapsible>
    )
  })
  .add('with header', () => {
    const header = <div>Collapsible Header</div>
    return (
      <Collapsible open={true} onToggle={actions.onToggle} header={header}>
        <div>Collapsible content</div>
        <div>
          Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum,
          tenetur eveniet. Adipisci sed, labore eos molestias.
        </div>
      </Collapsible>
    )
  })
  .add('interactive', () => {
    const model = observable({
      open: true,
      animate: true,
    })

    const onToggle = mobxAction(() => (model.open = !model.open))
    const onCheckboxChange = mobxAction((event: React.ChangeEvent<HTMLInputElement>) => {
      model.animate = event.target.checked
    })

    const header = <div>Click to toggle</div>
    const Component = observer(() => (
      <div>
        <label>
          <input type="checkbox" checked={model.animate} onChange={onCheckboxChange} /> Animate
        </label>
        <Collapsible open={model.open} onToggle={onToggle} header={header} animate={model.animate}>
          <div>Collapsible content</div>
          <div>
            Lorem ipsum dolor sit amet, consectetur adipisicing elit. Quis quam, beatae ut ipsum,
            tenetur eveniet. Adipisci sed, labore eos molestias.
          </div>
        </Collapsible>
      </div>
    ))

    return <Component />
  })
