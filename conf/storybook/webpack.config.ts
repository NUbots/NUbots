import * as HtmlWebpackPlugin from 'html-webpack-plugin'
import * as webpack from 'webpack'
import config from '../../webpack.config'

module.exports = (storybookConfig: webpack.Configuration) => {
  return {
    ...config,
    ...storybookConfig,
    module: {
      ...storybookConfig.module,
      rules: config.module && config.module.rules,
    },
    plugins: [
      ...storybookConfig.plugins || [],
      ...(config.plugins || []).filter(p => !(p instanceof HtmlWebpackPlugin)),
    ],
    resolve: {
      ...storybookConfig.resolve,
      extensions: [
        ...(storybookConfig.resolve && storybookConfig.resolve.extensions || []),
        ...(config.resolve && config.resolve.extensions || []),
      ],
    },
  }
}
