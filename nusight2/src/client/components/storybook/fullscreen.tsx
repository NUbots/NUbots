import React from 'react'
import { StoryApi } from '@storybook/addons'
import { StoryFnReactReturnType } from '@storybook/react/dist/ts3.9/client/preview/types'

export function fullscreenDecorator(story: () => JSX.Element) {
  return <div style={{ width: '100vw', height: '100vh' }}>{story()}</div>
}

export function fullscreen(stories: StoryApi<StoryFnReactReturnType>) {
  stories.addParameters({ layout: 'fullscreen' })
  stories.addDecorator(fullscreenDecorator)
  return stories
}
