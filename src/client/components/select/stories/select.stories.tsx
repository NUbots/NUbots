import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import { action as mobxAction, observable } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'

import { Option, Select } from '../view'

import Icon from './icon.svg'

const actions = {
  onChange: action('onChange'),
}

storiesOf('components.select', module)
  .addDecorator(story => <div style={{ maxWidth: '350px' }}>{story()}</div>)
  .add('renders basic', () => {
    return <Select options={[]} onChange={actions.onChange} placeholder="Select..." />
  })
  .add('renders empty', () => {
    const empty = (
      <div>
        <h4>No options</h4>
        <p>Add options to see them here</p>
      </div>
    )
    return <Select options={[]} onChange={actions.onChange} placeholder="Select..." empty={empty} />
  })
  .add('renders with options', () => {
    const options = getOptions()
    return <Select options={options} onChange={actions.onChange} placeholder="Select a color..." />
  })
  .add('renders with selection', () => {
    const options = getOptions()
    const selected = options[1]
    return (
      <Select
        options={options}
        selectedOption={selected}
        onChange={actions.onChange}
        placeholder="Select a color..."
      />
    )
  })
  .add('renders with icon', () => {
    const options = getOptions()
    return (
      <Select
        options={options}
        onChange={actions.onChange}
        placeholder="Select a color..."
        icon={<Icon />}
      />
    )
  })
  .add('interactive', () => {
    const options = getOptions()
    const model = observable({
      options,
      selectedOption: options[1],
    })
    const onChange = mobxAction((option: Option) => (model.selectedOption = option))
    const Component = observer(() => (
      <Select
        options={model.options}
        selectedOption={model.selectedOption}
        onChange={onChange}
        placeholder="Select a color..."
        icon={<Icon />}
      />
    ))

    return <Component />
  })

function getOptions(): Option[] {
  return [
    { id: 'red', label: 'Red' },
    { id: 'green', label: 'Green' },
    { id: 'blue', label: 'Blue' },
  ]
}
