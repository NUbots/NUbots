const storybook = require('@storybook/react/standalone')

storybook({
  mode: 'dev',
  port: process.env.PORT || 9001,
  configDir: __dirname,
})
