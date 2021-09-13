import minimist from 'minimist'
import * as path from 'path'
import webpack from 'webpack'
import { ConfigOptions, getClientConfig, getServerConfig } from '../webpack.config'

const args = minimist(process.argv.slice(2))
const isContinuousIntegration = args.ci || false
const isProduction = args.production || args.p || false

const opts: ConfigOptions = {
  mode: isProduction ? 'production' : 'development',
  context: path.join(__dirname, '..', 'src'),
  sourceMap: isContinuousIntegration ? false : 'source-map',
  rootDir: path.join(__dirname, '..'),
}

const compiler = webpack([getClientConfig(opts), getServerConfig(opts)])
compiler.run((err, stats) => {
  if (err) {
    process.stderr.write(err.message)
    if (err.stack) {
      process.stderr.write(err.stack)
    }
    process.exitCode = 1
    return
  }
  if (stats) {
    process.stdout.write(stats.toString({ colors: true }) + '\n')
    if (stats.hasErrors()) {
      process.exitCode = 1
    }
  }
  // Calling close() allows low-priority work (like persistent caching) the opportunity to complete
  compiler.close(closeErr => {
    if (closeErr) {
      process.stderr.write(closeErr.message)
      if (closeErr.stack) {
        process.stderr.write(closeErr.stack)
      }
      process.exitCode = 1
    }
  })
})
