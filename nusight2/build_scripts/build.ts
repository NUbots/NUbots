import minimist from 'minimist'
import * as path from 'path'
import webpack from 'webpack'
import { ConfigOptions } from '../webpack.config'
import { getServerConfig } from '../webpack.config'
import { getClientConfig } from '../webpack.config'

const args = minimist(process.argv.slice(2))
const isContinuousIntegration = args.ci || false
const isProduction = args.production || args.p || false

const opts: ConfigOptions = {
  mode: isProduction ? 'production' : 'development',
  context: path.join(__dirname, '..', 'src'),
  sourceMap: isContinuousIntegration ? false : 'source-map',
  rootDir: path.join(__dirname, '..'),
}
webpack([getClientConfig(opts), getServerConfig(opts)]).run((err, stats) => {
  process.stdout.write(stats.toString({ colors: true }) + '\n')
})
