import { addDecorator, configure } from '@storybook/react'

import { fontDecorator } from './decorators/font'

const req = require.context('../../src', true, /.stories.tsx$/)

addDecorator(fontDecorator)

function loadStories() {
  req.keys().forEach(req)
}

configure(loadStories, module)
