import React from 'react'

export function fullscreen(story: () => JSX.Element) {
  return <div style={{ width: '100vw', height: '100vh' }}>{story()}</div>
}
