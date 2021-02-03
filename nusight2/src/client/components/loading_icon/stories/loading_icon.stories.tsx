import { storiesOf } from '@storybook/react'
import React from 'react'

import { LoadingIcon } from '../view'

storiesOf('components.loading_icon', module)
  .add('renders animated', () => {
    return <LoadingIcon />
  })
  .add('renders with custom size', () => {
    return <LoadingIcon size={64} />
  })
