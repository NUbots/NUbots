import { RenderFunction } from '@storybook/react'
import * as React from 'react'

import * as style from './fullscreen.css'

export function fullscreen(story: RenderFunction) {
  return <div className={style.fullscreen}>
    {
      // This is an inline <style> and not in `fullscreen.css` as that would make it apply globally
      // to all stories, due to the way we build and extract the styles from CSS files.
    }
    <style>{`body { margin: 0; padding: 0; }`}</style>
    {story()}
  </div>
}
