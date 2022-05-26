import HtmlWebpackPlugin from 'html-webpack-plugin'
import * as path from 'path'
import webpack from 'webpack'
import { getClientConfig } from '../../webpack.config'

export default ({ config: storybookConfig }: { config: webpack.Configuration }) => {
  const config = getClientConfig({
    mode: 'development',
    context: path.resolve(path.join(__dirname, '..', '..', 'src')),
    sourceMap: 'eval-source-map',
    rootDir: path.join(__dirname, '..', '..'),
  })
  return {
    ...config,
    ...storybookConfig,
    module: {
      ...storybookConfig.module,
      rules: config.module?.rules,
    },
    plugins: [
      ...(storybookConfig.plugins || []),
      ...(config.plugins || []).filter(p => !(p instanceof HtmlWebpackPlugin)), // Storybook handles page generation.
    ],
    resolve: {
      ...storybookConfig.resolve,
      extensions: [
        ...(storybookConfig.resolve?.extensions || []),
        ...(config.resolve?.extensions || []),
      ],
    },
  }
}
