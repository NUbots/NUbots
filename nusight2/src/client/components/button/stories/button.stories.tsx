import { action } from '@storybook/addon-actions'
import { storiesOf } from '@storybook/react'
import React from 'react'

import { Button } from '../view'

import IconAfter from './icon-after.svg'
import IconBefore from './icon-before.svg'

const onClick = action('onClick')

const HSpacer = () => <span style={{ display: 'inline-block', width: '4px' }} />
const VSpacer = () => <div style={{ height: '4px' }} />

storiesOf('components.button', module)
  .add('renders basic', () => {
    return (
      <div>
        <Button onClick={onClick}>Normal</Button>
        <HSpacer />
        <Button type="primary" onClick={onClick}>
          Primary
        </Button>
      </div>
    )
  })
  .add('renders fullwidth', () => {
    return (
      <div>
        <Button fullwidth onClick={onClick}>
          Normal
        </Button>
        <VSpacer />
        <Button type="primary" fullwidth onClick={onClick}>
          Primary
        </Button>
      </div>
    )
  })
  .add('renders aligned left', () => {
    return (
      <div>
        <Button fullwidth textAlign="left" onClick={onClick}>
          Normal
        </Button>
        <VSpacer />
        <Button type="primary" textAlign="left" fullwidth onClick={onClick}>
          Primary
        </Button>
      </div>
    )
  })
  .add('renders aligned right', () => {
    return (
      <div>
        <Button fullwidth textAlign="right" onClick={onClick}>
          Normal
        </Button>
        <VSpacer />
        <Button type="primary" textAlign="right" fullwidth onClick={onClick}>
          Primary
        </Button>
      </div>
    )
  })
  .add('renders icon before', () => {
    return (
      <div>
        <Button iconBefore={<IconBefore />} onClick={onClick}>
          Button
        </Button>
        <VSpacer />
        <Button iconBefore={<IconBefore />} fullwidth onClick={onClick}>
          Fullwidth
        </Button>
        <VSpacer />
        <Button iconBefore={<IconBefore />} fullwidth textAlign="left" onClick={onClick}>
          Fullwidth, aligned left
        </Button>
        <VSpacer />
        <Button iconBefore={<IconBefore />} fullwidth textAlign="right" onClick={onClick}>
          Fullwidth, aligned right
        </Button>
      </div>
    )
  })
  .add('renders icon after', () => {
    return (
      <div>
        <Button iconAfter={<IconAfter />} onClick={onClick}>
          Button
        </Button>
        <VSpacer />
        <Button iconAfter={<IconAfter />} fullwidth onClick={onClick}>
          Button, fullwidth
        </Button>
        <VSpacer />
        <Button iconAfter={<IconAfter />} fullwidth textAlign="left" onClick={onClick}>
          Fullwidth, aligned left
        </Button>
        <VSpacer />
        <Button
          iconAfter={<IconAfter />}
          iconAfterAlignedRight
          fullwidth
          textAlign="left"
          onClick={onClick}
        >
          Fullwidth, aligned left, icon aligned right
        </Button>
        <VSpacer />
        <Button iconAfter={<IconAfter />} fullwidth textAlign="right" onClick={onClick}>
          Fullwidth, aligned right
        </Button>
      </div>
    )
  })
  .add('renders disabled', () => {
    return (
      <div>
        <Button disabled>Normal</Button>
        <HSpacer />
        <Button type="primary" disabled>
          Primary
        </Button>
      </div>
    )
  })
