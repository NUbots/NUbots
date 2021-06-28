declare module 'progress-bar-webpack-plugin' {
  import webpack from 'webpack'
  const value: new () => webpack.Plugin
  export default value
}
