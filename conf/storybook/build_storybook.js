const storybook = require('@storybook/react/standalone')

storybook({
  mode: 'static',
  staticDir: ['src/assets'],
  outputDir: 'dist/public/storybook',
  configDir: __dirname,
})
