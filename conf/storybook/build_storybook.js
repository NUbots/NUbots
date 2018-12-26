const storybook = require('@storybook/react/standalone')

storybook({
  mode: 'static',
  staticDir: ['src/assets'],
  outputDir: 'dist/storybook',
  configDir: __dirname,
})
