import { RenderFunction } from '@storybook/react'
import * as React from 'react'

import * as style from './font.css'

export function fontDecorator(story: RenderFunction) {
  return <div className={style.font}>{story()}</div>
}
